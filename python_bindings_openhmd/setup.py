#!/usr/bin/python

from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
 
rift_module = Extension("rift",
    ["pyrift.pyx", "Rift.cpp"],
    language="c++",
    libraries=["openhmd"],
    include_dirs=['/usr/include/openhmd', '/usr/local/include/openhmd'],
    extra_compile_args=["-std=c++11"],
    extra_link_args=["-std=c++11"]
    )
 
setup(name='rift',
      version='1.0',
      description='Python OpenHMD Wrapper',
      ext_modules=[rift_module],
      cmdclass={'build_ext': build_ext})
