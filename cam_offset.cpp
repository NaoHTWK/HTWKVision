#include <cam_offset.h>
#include <cstdio>

using namespace std;

float CamOffset::headPitch=0;
float CamOffset::headRoll=0;
int CamOffset::lowerCamOffsetX=0;
int CamOffset::lowerCamOffsetY=0;
int CamOffset::lowerCamOffsetCnt=0;
float CamOffset::bodyPitch=0;
float CamOffset::bodyRoll=0;

float CamOffset::deltaPitch[30];
float CamOffset::deltaRoll[30];
int CamOffset::deltaIdx=0;

void CamOffset::init(){
	bool paramsRead=false;
	FILE *fp=fopen("camOffset_v2.txt","r");
	if(fp!=nullptr){
		int read=fscanf(fp,"headPitch: %f\n"
		                   "headRoll: %f\n"
		                   "bodyPitch: %f\n"
		                   "bodyRoll: %f\n"
		                   "lowerCamOffsetX: %d\n"
		                   "lowerCamOffsetY: %d",
		                   &headPitch,&headRoll,&bodyPitch,&bodyRoll,&lowerCamOffsetX,&lowerCamOffsetY);
		if(read==6){
			paramsRead=true;
		}
		fclose(fp);
	}
	if(!paramsRead){
		headPitch=0;
		headRoll=0;
		bodyPitch=0;
		bodyRoll=0;
		lowerCamOffsetX=0;
		lowerCamOffsetY=0;
		lowerCamOffsetCnt=0;
		printf("-----------------------------------------\n"
               "|                                       |\n"
               "|            NO CAM OFFSETS!            |\n"
               "|       USE THE CALIBRATION AGENT!      |\n"
               "|                                       |\n"
		       "-----------------------------------------\n");
	}
}

void CamOffset::write(){
    FILE *fp=fopen(".camOffset_v2.txt","w");
    fprintf(fp,"headPitch: %f\n"
               "headRoll: %f\n"
               "bodyPitch: %f\n"
               "bodyRoll: %f\n"
               "lowerCamOffsetX: %d\n"
               "lowerCamOffsetY: %d",
               headPitch,headRoll,bodyPitch,bodyRoll,lowerCamOffsetX,lowerCamOffsetY);
    fflush(fp);
    fclose(fp);
    rename(".camOffset_v2.txt","camOffset_v2.txt");
}
