#!/home/fury/archiconda3/envs/gvins/bin/python

import rospy
from std_msgs.msg import Float32, Float64MultiArray
import sys
sys.path.insert(0, '/home/fury/zed_yolo')  # 根据YOLO代码所在路径进行调整
import numpy as np
import torch
import cv2
import pyzed.sl as sl
from threading import Lock, Thread
from time import sleep
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords
from utils.torch_utils import select_device
from utils.augmentations import letterbox
from utils.plots import Annotator

# 全局变量
lock = Lock()
run_signal = False
exit_signal = False
image_right = None

# 初始化Publisher
distance_pub = None
zhuitong_pub = None  # 新增锥桶坐标发布器

def torch_thread(weights, img_size, conf_thres=0.5, iou_thres=0.2, dnn=False, classes=None, agnostic_nms=False, max_det=1000, augment=False, visualize=False):
    global image_net, exit_signal, run_signal, distance_pub, point_cloud

    print("Initializing YOLO Network...")
    device = select_device()
    half = device.type != 'cpu'  # half precision only supported on CUDA
    imgsz = img_size

    # Load model
    model = attempt_load(weights, device=device)
    stride, names = model.stride.max(), model.names
    imgsz = check_img_size(imgsz, s=stride)
    if half:
        model.half()

    # Warmup model
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))

    while not exit_signal:
        if run_signal:
            lock.acquire()
            img, ratio, pad = letterbox(image_net[:, :, :3], imgsz, auto=False)
            img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(device)
            img = img.half() if half else img.float()
            img = img / 255.0
            if len(img.shape) == 3:
                img = img[None]
            
            pred = model(img, augment=augment, visualize=visualize)
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
            
            # 使用Annotator绘制检测结果
            annotator = Annotator(image_net, line_width=3, example=str(names))

            # 准备锥桶数据列表
            zhuitong_data = []

            for i, det in enumerate(pred):
                if len(det):
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], image_net.shape).round()
                    for *xyxy, conf, cls in reversed(det):
#                         label = f"{names[int(cls)]} {conf:.2f}"
#                         annotator.box_label(xyxy, label, color=(255, 0, 0))

#                         # 中心点坐标和距离
#                         cent_x = round((xyxy[0].item() + xyxy[2].item()) / 2)
#                         cent_y = round((xyxy[1].item() + xyxy[3].item()) / 2)

#                         #print(f"中心点坐标: ({cent_x}, {cent_y})")
# # 初始化默认距离值
#                         distance = 0.0  # 默认值

#                         point_cloud_value = point_cloud.get_value(cent_x, cent_y)[1]
#                         if point_cloud_value[2] > 0.0:
#                             distance = np.sqrt(point_cloud_value[0] ** 2 + point_cloud_value[1] ** 2 + point_cloud_value[2] ** 2)
#                             #rospy.loginfo(f"Object distance: {distance}")
#                             distance_pub.publish(distance)

#                              # 添加到锥桶数据 (使用标志位1表示所有锥桶)
#                         zhuitong_data.extend([1.0, float(cent_x), float(distance)])
                        detected_label = names[int(cls)]
                        label = f"{detected_label} {conf:.2f}"
                        annotator.box_label(xyxy, label, color=(255, 0, 0))

                        # 计算中心点坐标
                        cent_x = round((xyxy[0].item() + xyxy[2].item()) / 2)
                        cent_y = round((xyxy[1].item() + xyxy[3].item()) / 2)

                        # 默认距离值
                        distance = 0.0

                        point_cloud_value = point_cloud.get_value(cent_x, cent_y)[1]
                        if point_cloud_value[2] > 0.0:
                            distance = np.sqrt(point_cloud_value[0] ** 2 + point_cloud_value[1] ** 2 + point_cloud_value[2] ** 2)
                            distance_pub.publish(distance)

                        # 根据标签设置标志位
                        flag = 1.0  # 默认：red使用1
                        if detected_label.lower() == "blue":  # 如果标签为blue，则标志位改为2
                            flag = 2.0

                        # 添加到锥桶数据
                        zhuitong_data.extend([flag, float(cent_x), float(distance)])

                         # 发布锥桶坐标数据(新增功能)
            if zhuitong_data:
                msg = Float64MultiArray()
                msg.data = zhuitong_data
                zhuitong_pub.publish(msg)
                #rospy.loginfo(f"发布锥桶数据: 共{len(zhuitong_data)//3}个锥桶")


            # 获取绘制后的图像并显示
            output_image = annotator.result()
            cv2.imshow("YOLO Detection", output_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # 按下 'q' 键退出
                exit_signal = True
            
            lock.release()
            run_signal = False
        sleep(0.01)

def main():
    global image_net, exit_signal, run_signal, point_cloud
    rospy.init_node('zed_yolo_node', anonymous=True)
    
    global distance_pub, zhuitong_pub
    distance_pub = rospy.Publisher('/object_distance', Float32, queue_size=10)
    zhuitong_pub = rospy.Publisher('zhuitong_xy_topic', Float64MultiArray, queue_size=10)

    capture_thread = Thread(target=torch_thread,
                            kwargs={'weights': '/home/fury/fsd/fury_allv5.pt', 'img_size': 640, "conf_thres": 0.5})
    capture_thread.start()

    print("Initializing Camera...")
    zed = sl.Camera()
    init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720, coordinate_units=sl.UNIT.METER, depth_mode=sl.DEPTH_MODE.QUALITY)
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    image_left_tmp = sl.Mat()
    point_cloud = sl.Mat()

    while not rospy.is_shutdown() and not exit_signal:
        if zed.grab(sl.RuntimeParameters()) == sl.ERROR_CODE.SUCCESS:
            lock.acquire()
            zed.retrieve_image(image_left_tmp, sl.VIEW.LEFT)
            image_net = image_left_tmp.get_data()
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            lock.release()
            run_signal = True
            while run_signal:
                sleep(0.001)
        else:
            exit_signal = True
    exit_signal = True
    zed.close()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    with torch.no_grad():
        main()
