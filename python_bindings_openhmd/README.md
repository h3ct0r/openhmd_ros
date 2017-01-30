python-rift
===========

Python OpenHMD bindings for Oculus Rift, based on https://github.com/lubosz/python-rift

## Dependencies

* OpenHMD https://github.com/OpenHMD/OpenHMD
* Cython

## Install

```
$ ./setup.py install
```

## How to use it

```python
from rift import PyRift

def poll(hmd):
  hmd.poll()
  rotation = [hmd.rotation[0], hmd.rotation[1], hmd.rotation[2], hmd.rotation[3]]
  return rotation

rift = PyRift()

while(True):
	print poll(rift)
```

## Blog Post

* http://lubosz.wordpress.com/2013/06/26/oculus-rift-support-in-blender-game-engine/
