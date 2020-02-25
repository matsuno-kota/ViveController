
#ifndef VIVECONTROL_H_
#define VIVECONTROL_H_

#include <openvr.h>
#include <iostream>

//#include <opencv2\opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<glew.h>
#include<glfw3.h>


typedef struct{
	vr::HmdVector3_t position;
	vr::HmdQuaternion_t rotation;
	vr::HmdVector3_t velocity;
	vr::HmdVector3_t angularVelocity;
}ViveDeviceData;

typedef struct{
	ViveDeviceData pose;
	bool gripButton;
	bool applicationMenuButton;
	bool systemButton;
	float trigger;
	float padx;
	float pady;
}ControllerData;


class Vive
{
public:

	int initOpenGL(cv::Mat image);
	void render(cv::Mat image, ViveDeviceData* hmd, ControllerData* controller, ViveDeviceData* tracker);
	void finish();

	/*
	*	�R���X�g���N�^
	*	�����F�Ȃ�
	*	�Ԃ�l�F�Ȃ�
	*/
	Vive();

	/*
	*	Vive�̏�����
	*	�����F�Ȃ�
	*	�Ԃ�l�F�Ȃ�
	*/
	void openVive();

	/*
	*	Vive�̏I��
	*	�����F�Ȃ�
	*	�Ԃ�l�F�Ȃ�
	*/
	void closeVive();

	void countupConnectedDeviceNum();

	int returnNumHmd(){ return numHmd; }
	int returnNumController(){ return numController; }
	int returnNumTracker(){ return numTracker; }

	/*
	*	�S�Ẵf�o�C�X�̃f�[�^�̎擾
	*	�����F�ڑ�����Ă���S�Ẵf�o�C�X�̃f�[�^
	*	�Ԃ�l�F�Ȃ�
	*/
	void getAllViveData(ViveDeviceData* hmd, ControllerData* controller, ViveDeviceData* tracker);

	/*
	*	�C�x���g�̊m�F
	*	�����F�Ȃ�
	*	�Ԃ�l�F�Ȃ�
	*/
	void checkEvent();

private:
	vr::IVRSystem *m_pHMD;

	//�ڑ�����Ă���VIVE�f�o�C�X�̌�
	vr::TrackedDeviceIndex_t numDevice;
	int numHmd;
	int numController;
	int numTracker;

	//�R���g���[���̃{�^���E�p�b�h�E�g���K�[�̏�Ԃ̊i�[
	vr::VRControllerState_t controllerState;

	//VIVE�f�o�C�X�̈ʒu�p���E���x�̊i�[
	vr::TrackedDevicePose_t* trackedDevicePose;


	/*
	*	�ʒu�x�N�g���̎擾
	*	�����F�s��
	*	�Ԃ�l�F�Ȃ�
	*/
	vr::HmdVector3_t getPosition(vr::HmdMatrix34_t matrix);

	/*
	*	��]�N�I�[�e�[�V�����̎擾
	*	�����F�s��
	*	�Ԃ�l�F�Ȃ�
	*/
	vr::HmdQuaternion_t getRotation(vr::HmdMatrix34_t matrix);


	GLFWwindow* window;
	GLuint g_texID;
	uint32_t m_nRenderWidth, m_nRenderHeight;
};

#endif VIVECONTROL_H_ 