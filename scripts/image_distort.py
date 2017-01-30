import numpy as np
import cv2


class ImageLenseDistort:
    """
    Read frames from a camera and apply distortions
    """

    def __init__(self, parameters=None):
        self.params = None
        self.load_default_params()

        if parameters:
            for k in parameters.keys():
                self.params[k] = parameters[k]

        self.d_matrix = self.create_distortion_matrix(
            self.params['fxL'], self.params['cxL'], self.params['fyL'], self.params['cyL']
        )

    @staticmethod
    def create_distortion_matrix(_fx, _cx, _fy, _cy):
        """
        Construct distortion matrix from arguments
        """

        return np.array([
            [_fx, 0, _cx],
            [0, _fy, _cy],
            [0, 0, 1]
        ])

    def transform(self, img, k1=0.22, k2=0.24):
        """Apply barrel distortion using OpenCV's Undistort operation

        This counteracts the pincushion distortion that the Oculus lens
        applies. The distortion coefficients k1 and k2 are the main
        action here.

        [1]: http://docs.opencv.org/trunk/doc/py_tutorials/py_calib3d/\
                 py_calibration/py_calibration.html
        """
        return cv2.undistort(
            img,
            self.d_matrix,
            np.array([k1, k2, 0, 0, 0])
        )

    def translate(self, img, x, y):
        """Strict linear translation, using OpenCv's warpAffine

        TODO: Make rows and columns dynamic (OpenCV doesn't seem to  like
        it if you try to change this value during runtime).

        Also see the bottom of this page:
        http://www.3dtv.at/knowhow/EncodingDivx_en.aspx
        """
        columns, rows = self.params['width'], self.params['height']
        return cv2.warpAffine(
            img,
            np.float32([[1, 0, x], [0, 1, y]]),
            (columns, rows)
        )

    def crop(self, img, _xl, _xr, _yl, _yr):
        """Crop the image based on inputs

        For a color image, `image` is a 3D array, where the third dimenion
        is the RGB color separation. This method ignores the third
        dimensions, thus cropping all three equally.

        Args:
            image (np.array): The image matrix returned as the second
                item of cv2.VideoCapture.read().
            _xl, _xr, _yl, _yr (int): The boundaries to crop `image` to
        """

        width, height = self.params['width'], self.params['height']
        return img[
               _xl:width - _xr,
               _yl:height - _yr
               ]

    def load_default_params(self):
        self.params = {
            'cropXL': 0,
            'cropXR': 160,
            'cropYL': 0,
            'cropYR': 0,
            'cxL': 330,
            'cxR': 330,
            'cyL': 250,
            'cyR': 250,
            'fps': 15,
            'fxL': 270,
            'fxR': 270,
            'fyL': 360,
            'fyR': 360,
            'height': 480,
            'width': 720,
            'xL': 0,
            'xR': 0,
            'xo': 20,
            'xo2': -90,
            'yL': 0,
            'yR': 0,
            'yo': 0,
            'yo2': 0,
            'qcropXL': 0
        }

    def process_frame(self, img):
        """
        Process a frame, applies translations and
        distortions based on the parameter list.
        Then return the processed image with the 'barrel effect'
        """

        img = self.translate(img, self.params['xL'] + self.params['xo'], self.params['yL'] + self.params['yo'])
        img = self.transform(img)
        img = self.translate(img, self.params['xo2'], self.params['yo2'])

        img = self.crop(
            img,
            self.params['cropXL'],
            self.params['cropXR'],
            self.params['cropYL'],
            self.params['cropYR'],
        )

        return img
