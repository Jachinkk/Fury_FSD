import os
import re
import sys
import platform
import subprocess
import pathlib

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion
from io import open

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)

# https://stackoverflow.com/questions/50082055/python-setuptools-first-build-from-sources-then-install

class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                "CMake 2.8.6+ must be installed to build the following " +
                "extensions: " +
                ", ".join(e.name for e in self.extensions))

        cmake_version = LooseVersion(re.search(
            r'version\s*([\d.]+)',
            out.decode()
        ).group(1))
        if cmake_version < '2.8.6':
            raise RuntimeError("CMake >= 2.8.6 is required")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(
            self.get_ext_fullpath(ext.name)
        ))

        target_dir = os.path.join(self.build_temp,"target")
        self.distribution.target_dir = target_dir
        cmake_args = [
            '-DSWIG_IMPORT=ON',
            '-DWITH_PYTHON=ON',
            '-DWITH_EXAMPLES=OFF',
            '-DPYTHON_INCLUDE_DIR=' + self.include_dirs[0],
            '-DCMAKE_INSTALL_PREFIX=' + extdir,
            '-DWITH_SELFCONTAINED=ON',
            '-DWITH_DEEPBIND=ON'
        ]
        if sys.version_info[0] == 3:
          cmake_args.append('-DWITH_PYTHON3=ON')

        if "CASADI_SETUP_CMAKE_ARGS" in os.environ:
          cmake_args += os.environ["CASADI_SETUP_CMAKE_ARGS"].split("\n")

        self.announce("cmake_args (append more with CASADI_SETUP_CMAKE_ARGS env var, newline-seperated): ", level=3)
        for args in cmake_args:
          self.announce("  " + str(args), level=3)

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        env = os.environ.copy()
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        subprocess.check_call(
            ['cmake', ext.sourcedir] + cmake_args,
            cwd=self.build_temp,
            env=env
        )
        subprocess.check_call(
            ['cmake', '--build', '.'] + build_args + ["--target","install"],
            cwd=self.build_temp
        )


with open('./README.md', encoding='utf-8') as f:
    long_description = f.read()

with open('./requirements.txt') as f:
    install_requires = [line.strip('\n') for line in f.readlines()]

setup(
    name='casadi',
    version='3.5.5.1',
    author='Joel Andersson, Joris Gillis, Greg Horn',
    author_email='developer_first_name@casadi.org',
    maintainer='Joris Gillis',
    maintainer_email='joris@casadi.org',
    description=' CasADi -- framework for algorithmic differentiation and numeric optimization',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='http://casadi.org',
    ext_modules=[CMakeExtension('casadi')],
    cmdclass=dict(build_ext=CMakeBuild),
    install_requires=install_requires,
    zip_safe=False,
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Natural Language :: English',
        'Intended Audience :: Science/Research',
        'Operating System :: OS Independent',
        'Programming Language :: C++',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        ('License :: OSI Approved :: '
         'GNU Lesser General Public License v3 or later (LGPLv3+)'),
    ]
)
