#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <unordered_map>

#include "Rift.h"

Rift::Rift() {
  rotation = std::vector<float>(4);
  connect();
}

Rift::~Rift() {
  ohmd_ctx_destroy(ctx);
}

void Rift::connect() {
	ctx = ohmd_ctx_create();

	// Probe for devices
	num_devices = ohmd_ctx_probe(ctx);
	if(num_devices < 0){
		printf("[ERROR] failed to probe devices: %s\n", ohmd_ctx_get_error(ctx));
		return;
	}

	// Open default device (0)
	hmd = ohmd_list_open_device(ctx, 0);
	if(!hmd){
		printf("[ERROR] failed to open device: %s\n", ohmd_ctx_get_error(ctx));
		return;
	}
}

std::unordered_map<std::string, float> Rift::getDeviceInfo() {

	// Print hardware information for the opened device
	int ivals[2];
	ohmd_device_geti(hmd, OHMD_SCREEN_HORIZONTAL_RESOLUTION, ivals);
	ohmd_device_geti(hmd, OHMD_SCREEN_VERTICAL_RESOLUTION, ivals + 1);

	std::unordered_map<std::string, float> u = {
        {"num_devices_connected", num_devices},
        {"width_resolution", ivals[0]},
        {"height_resolution", ivals[1]},
        {"hsize", OHMD_SCREEN_HORIZONTAL_SIZE},
        {"vsize", OHMD_SCREEN_VERTICAL_SIZE},
        {"lens_separation", OHMD_LENS_HORIZONTAL_SEPARATION},
        {"lens_vcenter", OHMD_LENS_VERTICAL_POSITION},
        {"left_eye_fov", OHMD_LEFT_EYE_FOV},
        {"right_eye_fov", OHMD_RIGHT_EYE_FOV},
        {"left_eye_aspect", OHMD_LEFT_EYE_ASPECT_RATIO},
        {"left_eye_aspect", OHMD_RIGHT_EYE_ASPECT_RATIO},
        {"distortion_k", OHMD_DISTORTION_K}
    };

    return u;
}

std::vector<float> Rift::poll() {
    float rot[4];
	ohmd_ctx_update(ctx);
	ohmd_device_getf(hmd, OHMD_ROTATION_QUAT, rot);
	for(int i = 0; i < 4; i++) rotation[i] = rot[i];
	sleep(.01);

	return rotation;
}

void Rift::sleep(double seconds) {
	struct timespec sleepfor;
	sleepfor.tv_sec = (time_t)seconds;
	sleepfor.tv_nsec = (long)((seconds - sleepfor.tv_sec) * 1000000000.0);
	nanosleep(&sleepfor, NULL);
}

// gets float values from the device and prints them
void Rift::print(std::string name, int len, ohmd_float_value val) {
	float f[len];
	ohmd_device_getf(hmd, val, f);
	printf("%-20s", name.c_str());
	for(int i = 0; i < len; i++)
		printf("%f ", f[i]);
	printf("\n");
}

void Rift::reset(){
  ohmd_ctx_destroy(ctx);
  sleep(.1);
  connect();
}
