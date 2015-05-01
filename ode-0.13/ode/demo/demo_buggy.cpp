/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/
//sphere->cylinder
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif


#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif
////////////constans/////////
#define SQRT3 1.732
#define LENGTH 0.7
#define bot_LEN 0.6062//0.6062  LENGTH*SQRT3/2.0
	// chassis length
#define WIDTH 0.03
#define HEIGHT 0.03	// chassis height
#define WHEEL_WIDTH 0.3
#define RADIUS 0.18	// wheel radius
#define STARTZ 0.18	// starting height of chassis
#define CMASS 0.1		// chassis mass
#define WMASS 2	// wheel mass
#define BAR_LEN 0.45
#define pi 3.1415

// dynamics and collision objects (chassis, 3 wheels, environment)

static dWorldID world;
static dSpaceID space;
static dBodyID body[10];
static dJointID joint[10];	// joint[0] is the front wheel
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space;
static dGeomID box[6];
static dGeomID cylinder[3];
static dGeomID ground_box;


// things that the user controls

static dReal speed=0,steer=0;	// user commands



static void nearCallback (void *data, dGeomID o1, dGeomID o2){
  int i,n;

  // only collide things with the ground
  int g1 = (o1 == ground || o1 == ground_box);
  int g2 = (o2 == ground || o2 == ground_box);
  if (!(g1 ^ g2)) return;

  const int N = 10;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if (n > 0) {
    for (i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
	dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = dInfinity;
      contact[i].surface.slip1 = 0.1;
      contact[i].surface.slip2 = 0.1;
      contact[i].surface.soft_erp = 0.5;
      contact[i].surface.soft_cfm = 0.3;
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
		    dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));
    }
  }
}
static void start(){
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {3.8317f,-0.9817f,3.8000f};
  static float hpr[3] = {121.0000f,-30.5000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
  printf ("Press:\t'a' to increase speed.\n"
	  "\t'z' to decrease speed.\n"
	  "\t',' to steer left.\n"
	  "\t'.' to steer right.\n"
	  "\t' ' to reset speed and steering.\n"
	  "\t'1' to save the current state to 'state.dif'.\n");
}
static void command (int cmd){
  switch (cmd) {
  case 'a': case 'A':
    speed += 0.3;
    break;
  case 'z': case 'Z':
    speed -= 0.3;
    break;
  case ',':
    steer -= 0.5;
    break;
  case '.':
    steer += 0.5;
    break;
  case ' ':
    speed = 0;
    steer = 0;
    break;
  case '1': {
      FILE *f = fopen ("state.dif","wt");
      if (f) {
        dWorldExportDIF (world,f,"");
        fclose (f);
      }
    }
  }
}

static void simLoop (int pause)
{
  int i;
  if (!pause) {
	   const dReal *a0 = dBodyGetPosition (body[1]);//self
	const dReal *a1 = dBodyGetPosition (body[3]);
	const dReal *a2 = dBodyGetPosition (body[4]);
	const dReal *a3 = dBodyGetPosition (body[5]);
	//const dReal *deta= *a2-*a1;
	float tem1=a2[0]-a1[0];
	float tem2=a2[1]-a1[1];
	float tem3=a2[2]-a1[2];
	float tem4=a3[0]-a2[0];
	float tem5=a3[1]-a2[1];
	float tem6=a3[2]-a2[2];
	   dJointSetHinge2Axis1 (joint[0],tem1,tem2,tem3);
	//dJointSetHinge2Axis1 (joint[1],0,0,1);
   dJointSetHinge2Axis2 (joint[0],tem4,tem5,tem6);
    // motor
	 dJointSetHinge2Param (joint[0],dParamVel2,-speed);
    dJointSetHinge2Param (joint[0],dParamFMax2,0.1);

    // steering
    dReal v = steer - dJointGetHinge2Angle1 (joint[0]);
    if (v > 0.1) v = 0.1;
    if (v < -0.1) v = -0.1;
    v *= 10.0;
    dJointSetHingeParam (joint[2],dParamVel,v);
    dJointSetHingeParam (joint[2],dParamFMax,0.2);
    dJointSetHingeParam (joint[2],dParamLoStop,-0.75);
    dJointSetHingeParam (joint[2],dParamHiStop,0.75);
    dJointSetHingeParam (joint[2],dParamFudgeFactor,0.1);

    dSpaceCollide (space,0,&nearCallback);
    dWorldStep (world,0.05);

    // remove all contact joints
    dJointGroupEmpty (contactgroup);
   
 }

  dsSetColor (1,0,0);
  dsSetTexture (DS_WOOD);
  dReal sides[3] = {LENGTH,WIDTH,HEIGHT};
  dReal b_sides[3]={bot_LEN,WIDTH,HEIGHT};
  dReal bar_sides[3]={BAR_LEN,WIDTH,HEIGHT};
  dReal hand[3]={0.05,0.05,0.05};
  /////////////////////////////bot bar--body[0]////
  dsDrawBox (dBodyGetPosition(body[0]),dBodyGetRotation(body[0]),b_sides);//bot_line
  dsSetColor (1,0,0);
  dsDrawBox (dBodyGetPosition(body[3]),dBodyGetRotation(body[3]),sides);
  //////////////////////two bar=body[3],body[4]//////
  dsDrawBox (dBodyGetPosition(body[4]),dBodyGetRotation(body[4]),bar_sides);
  dsSetColor (0,0,1);
  dsDrawBox (dBodyGetPosition(body[5]),dBodyGetRotation(body[5]),hand);
  dsDrawBox (dBodyGetPosition(body[6]),dBodyGetRotation(body[6]),hand);
  //////////////////////two wheel-body[1],body[2]///////
  dsSetColor (0,1,0);
  for (i=1; i<=2; i++) dsDrawCylinder (dBodyGetPosition(body[i]),
				       dBodyGetRotation(body[i]),WHEEL_WIDTH,RADIUS);

  dVector3 ss;
  dGeomBoxGetLengths (ground_box,ss);
  dsDrawBox (dGeomGetPosition(ground_box),dGeomGetRotation(ground_box),ss);

  /*
  printf ("%.10f %.10f %.10f %.10f\n",
	  dJointGetHingeAngle (joint[1]),
	  dJointGetHingeAngle (joint[2]),
	  dJointGetHingeAngleRate (joint[1]),
	  dJointGetHingeAngleRate (joint[2]));
  */
}


int main (int argc, char **argv)
{
  int i;
  dMass m;
  dMatrix3 Rt;
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-0.5);
  ground = dCreatePlane (space,0,0,1,0);
 
  /////////////////////////////////////////
  ////////bar-shuzhede-body[3],box[1]//////
  /////////////////////////////////////////
  dRSetIdentity (Rt);
  dRFromAxisAndAngle (Rt,0,1,0,pi/3.0);
  body[3]=dBodyCreate(world);
  //dBodySetPosition (body[3],0.5*LENGTH-RADIUS/2.0-0.015,0,STARTZ+BAR_LEN*sin(pi/3.0)/2.0-0.015);//(body[1],0.5*LENGTH,0,STARTZ);
  dBodySetPosition (body[3],0.5*LENGTH-LENGTH/4.0,0,STARTZ+LENGTH*sin(pi/3.0)/2.0);
  dBodySetRotation (body[3],Rt);
  dMassSetBox (&m,1,LENGTH,WIDTH,HEIGHT);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[3],&m);
  box[1] = dCreateBox (0,BAR_LEN,WIDTH,HEIGHT);
  dGeomSetBody (box[1],body[3]);
  ///////////////////////////////////////////
  ////////////bar-hengzhede-body[4],box[2]///
  ///////////////////////////////////////////
  dRSetIdentity (Rt);
  dRFromAxisAndAngle (Rt,0,0,1,pi/2);//
  body[4]=dBodyCreate(world);
  const dReal *p0 = dBodyGetPosition (body[3]);
  float px0=p0[0];
  float pz0=p0[2];
  px0-=p0[0]/2.0;

  pz0+=p0[2]/2.0;
  dBodySetPosition(body[4],0.5*LENGTH-LENGTH/2.0,0,STARTZ+LENGTH*sin(pi/3.0));;//STARTZ+(BAR_LEN*sin(pi/3.0)/2.0-0.015)*2
  
  dBodySetRotation (body[4],Rt);
  dMassSetBox (&m,1,HEIGHT,WIDTH,BAR_LEN);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[4],&m);
  box[2] = dCreateBox (0,HEIGHT,WIDTH,BAR_LEN);
  dGeomSetBody (box[2],body[4]);
  /////////////////////////////////
  ////////hand_1/////////////////
  ///////////////////////////////
  body[5]=dBodyCreate(world);
  const dReal *p_bar = dBodyGetPosition (body[4]);

  dBodySetPosition(body[5],p_bar[0],p_bar[0]+BAR_LEN/2.0,p_bar[2]);;//STARTZ+(BAR_LEN*sin(pi/3.0)/2.0-0.015)*2
  dMassSetBox (&m,1,0.03,0.03,0.03);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[5],&m);
  box[3] = dCreateBox (0,0.03,0.03,0.03);
  dGeomSetBody (box[3],body[5]);
  /////////////////////////////////
  ////////hand_2/////////////////
  ///////////////////////////////
  body[6]=dBodyCreate(world);

  dBodySetPosition(body[6],p_bar[0],p_bar[0]-BAR_LEN/2.0,p_bar[2]);;//STARTZ+(BAR_LEN*sin(pi/3.0)/2.0-0.015)*2
  dMassSetBox (&m,1,0.03,0.03,0.03);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[6],&m);
  box[4] = dCreateBox (0,0.03,0.03,0.03);
  dGeomSetBody (box[4],body[6]);
  //////////////////////////////////////////////////
  /////// wheel bodies body[1]-body[2]//////////////
  /////////////////////////////////////////////////
  for (i=1; i<=2; i++) {
    body[i] = dBodyCreate (world);
    dQuaternion q;
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
    dBodySetQuaternion (body[i],q);
    dMassSetCylinder (&m,1,2,RADIUS,WHEEL_WIDTH);////////////
	
    dMassAdjust (&m,WMASS*i);
    dBodySetMass (body[i],&m);
    cylinder[i-1] = dCreateCCylinder(0,RADIUS,WHEEL_WIDTH);
    dGeomSetBody (cylinder[i-1],body[i]);
  }
  dBodySetPosition (body[1],0.5*LENGTH,0,STARTZ);//WIDTH*0.5
  dBodySetPosition (body[2],-0.5*LENGTH, 0,STARTZ);
    ////////////////////////////////////
  // chassis body body[0]-box[0]-bottom;
  ///////////////////////////////////////

  dRSetIdentity (Rt);
  dRFromAxisAndAngle (Rt,0,1,0,-pi/6.0);
  body[0] = dBodyCreate (world);
  dBodySetRotation (body[0],Rt);
 // const dReal *p0 = dBodyGetPosition (body[3]);
  const dReal *p1 = dBodyGetPosition (body[1]);
  float bot_temp0=p0[0]-p1[0];
  float bot_temp1=p0[1]-p1[1];
  float bot_temp2=p0[2]+p1[2];
 // dBodySetPosition (body[0],0.35*1.732-0.7,0,STARTZ+0.35*1.732/3);
  dBodySetPosition(body[0],bot_temp0/2.0,0,bot_temp2/2.0);
  dMassSetBox (&m,1,bot_LEN,WIDTH,HEIGHT);
  dMassAdjust (&m,CMASS);
  dBodySetMass (body[0],&m);
  box[0] = dCreateBox (0,bot_LEN,WIDTH,HEIGHT);
  dGeomSetBody (box[0],body[0]);

  /////////////////////////////////////////
  /////////hinge///////////////////////
  ///////////////////////////////////////
    const dReal *a0 = dBodyGetPosition (body[1]);//self
	const dReal *a1 = dBodyGetPosition (body[3]);
	const dReal *a2 = dBodyGetPosition (body[4]);
	const dReal *a3 = dBodyGetPosition (body[5]);
	//const dReal *deta= *a2-*a1;
	float tem1=a2[0]-a1[0];
	float tem2=a2[1]-a1[1];
	float tem3=a2[2]-a1[2];
	float tem4=a3[0]-a2[0];
	float tem5=a3[1]-a2[1];
	float tem6=a3[2]-a2[2];
  joint[2] = dJointCreateHinge(world,0);
  dJointAttach (joint[2] , body[3], body[0]);
  dJointSetHingeAxis (joint[2],tem1,tem2,tem3);
  const dReal *a_joint2 = dBodyGetPosition (body[3]);
  dJointSetHingeAnchor (joint[2],a_joint2[0],a_joint2[1],a_joint2[2]);

  //////////////////////////////////////////////////////////////////
  joint[3] = dJointCreateFixed (world,0);
  dJointAttach (joint[3] , body[3], body[4]);
  dJointSetFixed (joint[3] );
  
  joint[4] = dJointCreateFixed (world,0);
  dJointAttach (joint[4] , body[4], body[5]);
  dJointSetFixed (joint[4] );

  joint[5] = dJointCreateFixed (world,0);
  dJointAttach (joint[5] , body[4], body[6]);
  dJointSetFixed (joint[5] );
   //////////////////////////////////////
  ////front and back wheel hinges////////
  //////////////////////////////////////
    joint[1] = dJointCreateHinge2 (world,0);
    dJointAttach (joint[1],body[0],body[2]);
    const dReal *a = dBodyGetPosition (body[2]);
    dJointSetHinge2Anchor (joint[1],a[0],a[1],a[2]);
    dJointSetHinge2Axis1 (joint[1],0,0,1);
    dJointSetHinge2Axis2 (joint[1],0,1,0);
    dJointSetHinge2Param (joint[1],dParamSuspensionERP,0.4);
    dJointSetHinge2Param (joint[1],dParamSuspensionCFM,0.8);
    dJointSetHinge2Param (joint[1],dParamLoStop,0);
    dJointSetHinge2Param (joint[1],dParamHiStop,0);
	
	joint[0] = dJointCreateHinge2 (world,0);
    dJointAttach (joint[0],body[1],body[3]);
  
    dJointSetHinge2Anchor (joint[0],a0[0],a0[1],a0[2]);
   // dJointSetHinge2Axis1 (joint[0],tem1,tem2,tem3);
	//dJointSetHinge2Axis1 (joint[1],0,0,1);
   //dJointSetHinge2Axis2 (joint[0],tem4,tem5,tem6);
    dJointSetHinge2Param (joint[0],dParamSuspensionERP,0.4);
    dJointSetHinge2Param (joint[0],dParamSuspensionCFM,0.8);
    dJointSetHinge2Param (joint[0],dParamLoStop,0);
    dJointSetHinge2Param (joint[0],dParamHiStop,0);


    car_space = dSimpleSpaceCreate (space);
    dSpaceSetCleanup (car_space,0);
    dSpaceAdd (car_space,box[0]);
    dSpaceAdd (car_space,box[1]);
    dSpaceAdd (car_space,box[2]);
	dSpaceAdd (car_space,box[3]);
    dSpaceAdd (car_space,cylinder[0]);
  dSpaceAdd (car_space,cylinder[1]);
 

  // environment
  ground_box = dCreateBox (space,2,1.5,1);
  dMatrix3 R;
  dRFromAxisAndAngle (R,0,1,0,-0.15);
  dGeomSetPosition (ground_box,2,0,-0.34);
  dGeomSetRotation (ground_box,R);

  // run simulation
  dsSimulationLoop (argc,argv,352,288,&fn);

  dGeomDestroy (box[0]);
   dGeomDestroy (box[1]);
    dGeomDestroy (box[2]);
	dGeomDestroy (box[3]);
	dGeomDestroy (box[4]);
  dGeomDestroy (cylinder[0]);
  dGeomDestroy (cylinder[1]);

  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}
