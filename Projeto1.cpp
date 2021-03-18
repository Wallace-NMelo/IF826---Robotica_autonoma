/**
 * Controle do Robô de direção diferencial
 * Disciplina de Robótica CIn/UFPE
 * 
 * @autor Prof. Hansenclever Bassani
 * 
 * Este código é proporcionado para facilitar os passos iniciais da programação.
 * 
 * Testado em: Pop_OS 20.04
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <unistd.h>

#define COPPELIA_SIM_IP_ADDRESS /*"10.0.2.2"*/"127.0.0.1"
#define COPPELIA_SIM_PORT 19997//1999;

#define RAIO 0.02 //wheel radius
#define L 0.1 //wheels separation

extern "C" {
#include "extApi.h"
    /*	#include "extApiCustom.h" if you wanna use custom remote API functions! */
}

simxInt ddRobotHandle;
simxInt leftMotorHandle;
simxInt rightMotorHandle;
simxInt sensorHandle;
simxInt targetHandle;

void getPosition(int clientID, simxInt objectHandle, simxFloat pos[]) { //[x,y,theta]

    simxInt ret = simxGetObjectPosition(clientID, objectHandle, -1, pos, simx_opmode_oneshot_wait);
    if (ret > 0) {
        printf("Error reading object position\n");
        return;
    }

    simxFloat orientation[3];
    ret = simxGetObjectOrientation(clientID, objectHandle, -1, orientation, simx_opmode_oneshot_wait);
    if (ret > 0) {
        printf("Error reading object orientation\n");
        return;
    }

    simxFloat theta = orientation[2];
    pos[2] = theta;
}

simxInt getSimTimeMs(int clientID) { //In Miliseconds
    return simxGetLastCmdTime(clientID);
}

void setTargetSpeed(int clientID, simxFloat phiL, simxFloat phiR) {
    simxSetJointTargetVelocity(clientID, leftMotorHandle, phiL, simx_opmode_oneshot);
    simxSetJointTargetVelocity(clientID, rightMotorHandle, phiR, simx_opmode_oneshot);   
}

inline double to_deg(double radians) {
    return radians * (180.0 / M_PI);
}

void controle_movimento(simxFloat pos[3], simxFloat goal[3], simxFloat* PhiL, simxFloat* PhiR) {
	float theta = pos[2];
    float dx = goal[0] - pos[0];
    float dy = goal[1] - pos[1];
	int costa = 0;
	float rho, alpha, beta, kp=1000, ka=8100, kb= -100;
	rho = sqrt(dx*dx + dy*dy);	
	beta = -(goal[2]-pos[2]);
	printf("real theta = %f\n", pos[2]);
	alpha = -theta + atan2(dy, dx);
	if(theta> -M_PI_2 && theta <= M_PI_2 ){
		if(alpha> -M_PI_2 && alpha <= M_PI_2 ){
			
		}else{
			dy = -dy;
			dx= -dx;
			costa = 1;
		}
	}else{
		printf("\ncaiu aq\n");
		if(theta > 0){
			theta = -(M_PI - theta);
		}else{
			theta = -(M_PI + theta);
		}
		
		if(pos[2] > -M_PI && pos[2] <= -M_PI_2){
			theta = -theta;
		}
		
		dy = -dy;
		dx= -dx;
		alpha = -theta + atan2(dy, dx);
		if(alpha> -M_PI_2 && alpha <= M_PI_2 ){
		}else{
			costa = 1;
		}
	}
    alpha = -theta + atan2(dy, dx);
	
	if(costa == 1){
		beta = -beta;
		alpha = -alpha;
	}
	
	if(abs(beta) < 0.09){
		beta = 0;
	}
	if(rho < 0.05){
		rho = 0;
		alpha = 0;
	}
	float v = kp * rho;
	
	
	if(rho < 0.05){
		v = 0;
		alpha = 0;
	}
    float w = ka*alpha + kb*beta;
    	
	
	printf("\n dx = %f; dy = %f;alpha = %f; rho = %f ; beta = %f; theta = %f ; goal = %f ; atan = %f\n",dx,dy,alpha , rho, beta,theta, goal[2], atan2(dy,dx));
	
    *PhiR = (2*v+w*L)/2*RAIO;
    *PhiL = (2*v-w*L)/2*RAIO;
	
	if(costa == 1){
		*PhiR = (-2*v-w*L)/2*RAIO;
		*PhiL = (-2*v+w*L)/2*RAIO;
	}
	
	printf("\costa = %d; w = %f, beta = %f, kb = %f; termo errado = %f\n", costa, w, beta, kb, ka*alpha);
	
	//*PhiR = 0;
    //*PhiL = 0;
} 

int main(int argc, char* argv[]) {

    std::string ipAddr = COPPELIA_SIM_IP_ADDRESS;
    int portNb = COPPELIA_SIM_PORT;

    if (argc > 1) {
        ipAddr = argv[1];
    }

    printf("Iniciando conexao com: %s...\n", ipAddr.c_str());

    //Connect to the server
    int clientID = simxStart((simxChar*) (simxChar*) ipAddr.c_str(), portNb, true, true, 2000, 5);
    if (clientID != -1) {
        printf("Conexao efetuada\n");
        
        //Get handles for robot parts, actuators and sensores:
        simxGetObjectHandle(clientID, "RobotFrame#", &ddRobotHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "LeftMotor#", &leftMotorHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "RightMotor#", &rightMotorHandle, simx_opmode_oneshot_wait);
        simxGetObjectHandle(clientID, "Target#", &targetHandle, simx_opmode_oneshot_wait);
        
        printf("RobotFrame: %d\n", ddRobotHandle);
        printf("LeftMotor: %d\n", leftMotorHandle);
        printf("RightMotor: %d\n", rightMotorHandle);

        //start simulation
        int ret = simxStartSimulation(clientID, simx_opmode_oneshot_wait);
        
        if (ret==-1) {
            printf("Não foi possível iniciar a simulação.\n");
            return -1;
        }
        
        printf("Simulação iniciada.\n");

        //While is connected:
        while (simxGetConnectionId(clientID) != -1) {
            
            //Read current position:
            simxFloat pos[3]; //[x,y,theta] in [cm cm rad]
            getPosition(clientID, ddRobotHandle, pos);

            //Read simulation time of the last command:
            simxInt time = getSimTimeMs(clientID); //Simulation time in ms or 0 if sim is not running
            //stop the loop if simulation is has been stopped:
            if (time == 0) break;             
            printf("Posicao: [%.2f %.2f %.2f°] ", pos[0], pos[1], to_deg(pos[2]));
            
            //Set new target speeds: robot going in a circle:
            simxFloat goal[3]; 
            getPosition(clientID, targetHandle, goal);
            printf("Objetivo: [%.2f %.2f %.2f°]", goal[0], goal[1], to_deg(goal[2]));
            
            simxFloat phiL, phiR; //rad/s
            controle_movimento(pos, goal, &phiL, &phiR);
            
            setTargetSpeed(clientID, phiL, phiR);

            //Leave some time for CoppeliaSim do it's work:
            extApi_sleepMs(1);
            printf("\n");
        }
        
        //Stop the robot and disconnect from CoppeliaSim;
        setTargetSpeed(clientID, 0, 0);
        simxPauseSimulation(clientID, simx_opmode_oneshot_wait);
        simxFinish(clientID);
        
    } else {
        printf("Nao foi possivel conectar.\n");
        return -2;
    }
    
    return 0;
}


