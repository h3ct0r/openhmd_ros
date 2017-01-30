from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp.unordered_map cimport unordered_map
from libcpp.pair cimport pair

cdef extern from "Rift.h":
    cdef cppclass Rift:
        Rift()
        
        vector[float] rotation

        unordered_map[string, float] getDeviceInfo()
        vector[float] poll()
        void reset()
        

cdef class PyRift:
    cdef Rift* thisptr      # hold a C++ instance which we're wrapping

    def __cinit__(self):
        self.thisptr = new Rift()

    def __dealloc__(self):
        del self.thisptr

    def poll(self):
        return self.thisptr.poll()

    def reset(self):
        self.thisptr.reset()

    def getDeviceInfo(self):
        return self.thisptr.getDeviceInfo()

    property rotation:
      def __get__(self): return self.thisptr.rotation
