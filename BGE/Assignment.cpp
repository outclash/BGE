#include "PhysicsGame1.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

#include "PhysicsFactory.h"
#include "Game.h" 
#include "Model.h"
#include "dirent.h"
#include "Capsule.h" 

#include "Assignment.h"

using namespace BGE;

Assignment::Assignment(void)
{
}

Assignment::~Assignment(void)
{
}

bool Assignment::Initialise()
{

	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	dynamicsWorld->setGravity(btVector3(0, -9, 0));
	CreateProcedural();
	M1 = CreateM1();
	Ball = CreateBall();

	if (!Game::Initialise()) {
		return false;
	}

	return true;
}

void BGE::Assignment::Update(float timeDelta)
{
	/*if (keyState[SDL_SCANCODE_UP])
	{
		bot->transform->position += bot->transform->look * speed * timeDelta;
	}
	if (keyState[SDL_SCANCODE_DOWN])
	{
		bot->transform->position -= bot->transform->look * speed * timeDelta;
	}*/
	/*if (keyState[SDL_SCANCODE_X])
	{
		Ball->transform->position.y += 1;
		bot->transform->position += glm::vec3(0,-5,0);
		bot->rigidBody->applyCentralForce(btVector3(0,-5,0));
	}*/
	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}

shared_ptr<PhysicsController> Assignment::CreateM1()
{
	glm::quat faceSide = glm::angleAxis(90.0F, glm::vec3(0, 0, 1));
	glm::quat faceFront = glm::angleAxis(90.0F, glm::vec3(1, 0, 0));

	float pivot = 3;
	float boxR = 5;
	float boxH = 1;
	float boxD = 5;
	float boxX = -30;
	float boxY = 30;
	float boxZ = sR;
	float coreR = 5;
	float capH = 2;
	float capR = 1;

	shared_ptr<PhysicsController> core = physicsFactory->CreateSphere(coreR, glm::vec3(boxX, boxY + 5, boxZ), glm::quat()); 	//core
	shared_ptr<PhysicsController> bot = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY, boxZ), glm::quat()); //bottom
	//shared_ptr<PhysicsController> sBot = physicsFactory->CreateSphere(1, glm::vec3(boxX, boxY + 4, boxZ), glm::quat()); //bottom slide

	boxY += 5;
	shared_ptr<PhysicsController> botL = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX - 5, boxY, boxZ), faceSide); // bottom left
	shared_ptr<PhysicsController> botR = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX + 5, boxY, boxZ), faceSide); //bottom right
	shared_ptr<PhysicsController> botB = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY, boxZ - 5), faceFront); //bottom back
	shared_ptr<PhysicsController> botF = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY, boxZ + 5), faceFront); //bottom front

	boxY += 7.5f;
	shared_ptr<PhysicsController> sLeft = physicsFactory->CreateSphere(1, glm::vec3(boxX - 3, boxY, boxZ), glm::quat()); //left slide
	shared_ptr<PhysicsController> sRight = physicsFactory->CreateSphere(1, glm::vec3(boxX + 3, boxY, boxZ), glm::quat()); //right slide
	shared_ptr<PhysicsController> sBack = physicsFactory->CreateSphere(1, glm::vec3(boxX, boxY, boxZ - 3), glm::quat()); //back slide
	shared_ptr<PhysicsController> sFront = physicsFactory->CreateSphere(1, glm::vec3(boxX, boxY, boxZ + 3), glm::quat()); //front slide

	shared_ptr<PhysicsController> midL = physicsFactory->CreateBox(boxH, boxR, boxD, glm::vec3(boxX - 5, boxY, boxZ), glm::quat()); //mid left 
	shared_ptr<PhysicsController> midR = physicsFactory->CreateBox(boxH, boxR, boxD, glm::vec3(boxX + 5, boxY, boxZ), glm::quat()); //mid right
	shared_ptr<PhysicsController> midB = physicsFactory->CreateBox(boxR, boxD, boxH, glm::vec3(boxX, boxY, boxZ - 5), glm::quat()); //mid back
	shared_ptr<PhysicsController> midF = physicsFactory->CreateBox(boxR, boxD, boxH, glm::vec3(boxX, boxY, boxZ + 5), glm::quat()); //mid front

	shared_ptr<PhysicsController> top = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY + 12.5f, boxZ), glm::quat()); //top
	shared_ptr<PhysicsController> sTop = physicsFactory->CreateSphere(1, glm::vec3(boxX, boxY + 10, boxZ), glm::quat()); //top slide
	boxY += 7.5f;
	shared_ptr<PhysicsController> topL = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX - 5, boxY, boxZ), faceSide); // topleft
	shared_ptr<PhysicsController> topR = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX + 5, boxY, boxZ), faceSide);	//top right
	shared_ptr<PhysicsController> topF = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY, boxZ + 5), faceFront); //top front
	shared_ptr<PhysicsController> topB = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY, boxZ - 5), faceFront); //top back 
																														
	//ball point joint capsule_capsule (3 capsules) 1
	shared_ptr<PhysicsController> cp1 = physicsFactory->CreateCapsule(capR, capH/2, glm::vec3(boxX, boxY, boxZ), glm::quat());
	shared_ptr<PhysicsController> cp2 = physicsFactory->CreateCapsule(capR, capH*2, glm::vec3(boxX, 0 + 5, boxZ), faceSide);
	//shared_ptr<PhysicsController> cp3 = physicsFactory->CreateCapsule(capR, capH, glm::vec3(boxX, 0 + 5, boxZ), glm::quat());
	btPoint2PointConstraint * cp1_cp2 = new btPoint2PointConstraint(*cp1->rigidBody, *cp2->rigidBody, btVector3(0, (capH ) + 1, 0), btVector3(capR/2, 0, 0));
	dynamicsWorld->addConstraint(cp1_cp2);
	//btPoint2PointConstraint * cp1_cp3 = new btPoint2PointConstraint(*cp1->rigidBody, *cp3->rigidBody, btVector3(0, (capH * 2) + 1, 0), btVector3(2, -(capH ) + 1, 0));
	//dynamicsWorld->addConstraint(cp1_cp3);

	btTransform t1, t2;
	//t1.setIdentity();
	//t2.setIdentity();
	//t1.setOrigin(btVector3(0, -6, 0));
	//t2.setOrigin(btVector3(0, 0, 0));
	//btFixedConstraint * core_box = new btFixedConstraint(*core->rigidBody, *sBot->rigidBody, t1, t2); //core_sBot slide(fixed RN)
	//dynamicsWorld->addConstraint(core_box);

	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, capH*2, 0));
	t2.setOrigin(btVector3(0, 0, 0));
	btFixedConstraint * top_cps = new btFixedConstraint(*top->rigidBody, *cp1->rigidBody, t1, t2); //capsules_top
	dynamicsWorld->addConstraint(top_cps);


	//t1.setOrigin(btVector3(0, -2, 0));
	//core_box = new btFixedConstraint(*sBot->rigidBody, *bot->rigidBody, t1, t2);  //bottom box_sBot fixed joint
	//dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 6, 0));
	btFixedConstraint *core_box = new btFixedConstraint(*core->rigidBody, *sTop->rigidBody, t1, t2); //core_sTop (fixed rn)
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, -6, 0));
	core_box = new btFixedConstraint(*core->rigidBody, *bot->rigidBody, t1, t2);  //bottom box_core fixed joint
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 2, 0));
	core_box = new btFixedConstraint(*sTop->rigidBody, *top->rigidBody, t1, t2); // top box_sTop fixed joint
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(6, 0, 0));
	core_box = new btFixedConstraint(*core->rigidBody, *sRight->rigidBody, t1, t2); //core_sRight (fixed rn)
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(2, 0, 0));
	core_box = new btFixedConstraint(*sRight->rigidBody, *midR->rigidBody, t1, t2); // midR_sRight fixed joint
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(-6, 0, 0));
	core_box = new btFixedConstraint(*core->rigidBody, *sLeft->rigidBody, t1, t2); //core_sLeft(fixed rn)
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(-2, 0, 0));
	core_box = new btFixedConstraint(*sLeft->rigidBody, *midL->rigidBody, t1, t2); // midL_sLeft fixed joint
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 0, 6));
	core_box = new btFixedConstraint(*core->rigidBody, *sFront->rigidBody, t1, t2); //core_sFront (fixed rn)
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 0, 2));
	core_box = new btFixedConstraint(*sFront->rigidBody, *midF->rigidBody, t1, t2); // midF_sFront fixed joint
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 0, -6));
	core_box = new btFixedConstraint(*core->rigidBody, *sBack->rigidBody, t1, t2); //core_sBack (fixed rn)
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 0, -2));
	core_box = new btFixedConstraint(*sBack->rigidBody, *midB->rigidBody, t1, t2); // midB_sBack fixed joint
	dynamicsWorld->addConstraint(core_box);

	//bot_botL hinge
	btHingeConstraint * join = new btHingeConstraint(*bot->rigidBody, *botL->rigidBody, btVector3(-pivot, 0, 0), btVector3(pivot, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	join->enableAngularMotor(true,1,100);
	dynamicsWorld->addConstraint(join);

	//bot_botR
	join = new btHingeConstraint(*bot->rigidBody, *botR->rigidBody, btVector3(pivot, 0, 0), btVector3(pivot, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	join->enableAngularMotor(true, -1, 100);
	dynamicsWorld->addConstraint(join);

	//bot_botF
	join = new btHingeConstraint(*bot->rigidBody, *botF->rigidBody, btVector3(0, 0, pivot), btVector3(0, 0, pivot), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	join->enableAngularMotor(true, 1, 100);
	dynamicsWorld->addConstraint(join);

	//bot_botB
	join = new btHingeConstraint(*bot->rigidBody, *botB->rigidBody, btVector3(0, 0, -pivot), btVector3(0, 0, pivot), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	join->enableAngularMotor(true, -1, 100);
	dynamicsWorld->addConstraint(join);

	////botR_midR
	//join = new btHingeConstraint(*botR->rigidBody, *midR->rigidBody, btVector3(-pivot, 0, 0), btVector3(0, -pivot, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	//dynamicsWorld->addConstraint(join);

	////botL_midL
	//join = new btHingeConstraint(*botL->rigidBody, *midL->rigidBody, btVector3(-pivot, 0, 0), btVector3(0, -pivot, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	//dynamicsWorld->addConstraint(join);

	////botB_midB
	//join = new btHingeConstraint(*botB->rigidBody, *midB->rigidBody, btVector3(0, 0, -pivot), btVector3(0, -pivot, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	//dynamicsWorld->addConstraint(join);

	////botF_midF
	//join = new btHingeConstraint(*botF->rigidBody, *midF->rigidBody, btVector3(0, 0, -pivot), btVector3(0, -pivot, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	//dynamicsWorld->addConstraint(join);

	//topB_midB
	join = new btHingeConstraint(*topB->rigidBody, *midB->rigidBody, btVector3(0, 0, pivot), btVector3(0, pivot, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	//topF_midF
	join = new btHingeConstraint(*topF->rigidBody, *midF->rigidBody, btVector3(0, 0, pivot), btVector3(0, pivot, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	//topL_midL
	join = new btHingeConstraint(*topL->rigidBody, *midL->rigidBody, btVector3(pivot, 0, 0), btVector3(0, pivot, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//topR_midR
	join = new btHingeConstraint(*topR->rigidBody, *midR->rigidBody, btVector3(pivot, 0, 0), btVector3(0, pivot, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//top_topL
	join = new btHingeConstraint(*top->rigidBody, *topL->rigidBody, btVector3(-pivot, 0, 0), btVector3(-pivot, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//top_topR
	join = new btHingeConstraint(*top->rigidBody, *topR->rigidBody, btVector3(pivot, 0, 0), btVector3(-pivot, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//top_topF
	join = new btHingeConstraint(*top->rigidBody, *topF->rigidBody, btVector3(0, 0, pivot), btVector3(0, 0, -pivot), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	//top_topB
	join = new btHingeConstraint(*top->rigidBody, *topB->rigidBody, btVector3(0, 0, -pivot), btVector3(0, 0, -pivot), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	return core;
}

void BGE::Assignment::CreateProcedural()
{
	float r = 5;
	float height = 5;
	int num = 3;
	int hMultiplier = 0;
	sR = 0;

	while (num - 1> 0) { //num -1
		for (int i = 0; i < num; i++) {
			for (int j = 0; j < num; j++) {
				physicsFactory->CreateCylinder(r, height, glm::vec3(((r * 2)*i) + sR, height*hMultiplier, ((r * 2)*j) + sR), glm::quat());
			}
		}
		num--;
		hMultiplier++;
		sR += r;
		if (num == 1) {
		physicsFactory->CreateSphere(r, glm::vec3(sR, (height*hMultiplier) + r , sR), glm::quat());
		}
	}
}

shared_ptr<PhysicsController> Assignment::CreateBall()
{
	glm::quat faceSide = glm::angleAxis(90.0F, glm::vec3(0, 0, 1));
	glm::quat faceFront = glm::angleAxis(90.0F, glm::vec3(1, 0, 0));

	float pivot = 3;
	float boxR = 5;
	float boxH = 1;
	float boxD = 5;
	float boxX = 50;
	float boxY = 30;
	float boxZ = sR;
	float coreR = 5;
	float capH = 2;
	float capR = 1;

	shared_ptr<PhysicsController> core = physicsFactory->CreateSphere(coreR, glm::vec3(boxX, boxY + 5, boxZ), glm::quat()); 	//core
	shared_ptr<PhysicsController> bot = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY, boxZ), glm::quat()); //bottom
	shared_ptr<PhysicsController> sBot = physicsFactory->CreateSphere(1, glm::vec3(boxX, boxY + 4, boxZ), glm::quat()); //bottom slide

	boxY += 5;
	shared_ptr<PhysicsController> botL = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX - 5, boxY, boxZ), faceSide); // bottom left
	shared_ptr<PhysicsController> botR = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX + 5, boxY, boxZ), faceSide); //bottom right
	shared_ptr<PhysicsController> botB = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY, boxZ - 5), faceFront); //bottom back
	shared_ptr<PhysicsController> botF = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY, boxZ + 5), faceFront); //bottom front

	boxY += 7.5f;
	shared_ptr<PhysicsController> sLeft = physicsFactory->CreateSphere(1, glm::vec3(boxX - 3, boxY, boxZ), glm::quat()); //left slide
	shared_ptr<PhysicsController> sRight = physicsFactory->CreateSphere(1, glm::vec3(boxX + 3, boxY, boxZ), glm::quat()); //right slide
	shared_ptr<PhysicsController> sBack = physicsFactory->CreateSphere(1, glm::vec3(boxX, boxY, boxZ - 3), glm::quat()); //back slide
	shared_ptr<PhysicsController> sFront = physicsFactory->CreateSphere(1, glm::vec3(boxX, boxY, boxZ + 3), glm::quat()); //front slide

	shared_ptr<PhysicsController> midL = physicsFactory->CreateBox(boxH, boxR, boxD, glm::vec3(boxX - 5, boxY, boxZ), glm::quat()); //mid left 
	shared_ptr<PhysicsController> midR = physicsFactory->CreateBox(boxH, boxR, boxD, glm::vec3(boxX + 5, boxY, boxZ), glm::quat()); //mid right
	shared_ptr<PhysicsController> midB = physicsFactory->CreateBox(boxR, boxD, boxH, glm::vec3(boxX, boxY, boxZ - 5), glm::quat()); //mid back
	shared_ptr<PhysicsController> midF = physicsFactory->CreateBox(boxR, boxD, boxH, glm::vec3(boxX, boxY, boxZ + 5), glm::quat()); //mid front

	shared_ptr<PhysicsController> top = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY + 12.5f, boxZ), glm::quat()); //top
	shared_ptr<PhysicsController> sTop = physicsFactory->CreateSphere(1, glm::vec3(boxX, boxY + 10, boxZ), glm::quat()); //top slide
	boxY += 7.5f;
	shared_ptr<PhysicsController> topL = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX - 5, boxY, boxZ), faceSide); // topleft
	shared_ptr<PhysicsController> topR = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX + 5, boxY, boxZ), faceSide);	//top right
	shared_ptr<PhysicsController> topF = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY, boxZ + 5), faceFront); //top front
	shared_ptr<PhysicsController> topB = physicsFactory->CreateBox(boxR, boxH, boxD, glm::vec3(boxX, boxY, boxZ - 5), faceFront); //top back 


	btTransform t1, t2;
	t1.setIdentity();
	t2.setIdentity();
	t1.setOrigin(btVector3(0, -6, 0));
	t2.setOrigin(btVector3(0, 0, 0));
	btFixedConstraint * core_box = new btFixedConstraint(*core->rigidBody, *sBot->rigidBody, t1, t2); //core_sBot slide(fixed RN)
	dynamicsWorld->addConstraint(core_box);


	t1.setOrigin(btVector3(0, -2, 0));
	core_box = new btFixedConstraint(*sBot->rigidBody, *bot->rigidBody, t1, t2);  //bottom box_sBot fixed joint
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 6, 0));
	core_box = new btFixedConstraint(*core->rigidBody, *sTop->rigidBody, t1, t2); //core_sTop (fixed rn)
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, -6, 0));
	core_box = new btFixedConstraint(*core->rigidBody, *bot->rigidBody, t1, t2);  //bottom box_core fixed joint
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 2, 0));
	core_box = new btFixedConstraint(*sTop->rigidBody, *top->rigidBody, t1, t2); // top box_sTop fixed joint
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(6, 0, 0));
	core_box = new btFixedConstraint(*core->rigidBody, *sRight->rigidBody, t1, t2); //core_sRight (fixed rn)
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(2, 0, 0));
	core_box = new btFixedConstraint(*sRight->rigidBody, *midR->rigidBody, t1, t2); // midR_sRight fixed joint
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(-6, 0, 0));
	core_box = new btFixedConstraint(*core->rigidBody, *sLeft->rigidBody, t1, t2); //core_sLeft(fixed rn)
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(-2, 0, 0));
	core_box = new btFixedConstraint(*sLeft->rigidBody, *midL->rigidBody, t1, t2); // midL_sLeft fixed joint
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 0, 6));
	core_box = new btFixedConstraint(*core->rigidBody, *sFront->rigidBody, t1, t2); //core_sFront (fixed rn)
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 0, 2));
	core_box = new btFixedConstraint(*sFront->rigidBody, *midF->rigidBody, t1, t2); // midF_sFront fixed joint
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 0, -6));
	core_box = new btFixedConstraint(*core->rigidBody, *sBack->rigidBody, t1, t2); //core_sBack (fixed rn)
	dynamicsWorld->addConstraint(core_box);

	t1.setOrigin(btVector3(0, 0, -2));
	core_box = new btFixedConstraint(*sBack->rigidBody, *midB->rigidBody, t1, t2); // midB_sBack fixed joint
	dynamicsWorld->addConstraint(core_box);

	//bot_botL hinge
	btHingeConstraint * join = new btHingeConstraint(*bot->rigidBody, *botL->rigidBody, btVector3(-pivot, 0, 0), btVector3(pivot, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//bot_botR
	join = new btHingeConstraint(*bot->rigidBody, *botR->rigidBody, btVector3(pivot, 0, 0), btVector3(pivot, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//bot_botF
	join = new btHingeConstraint(*bot->rigidBody, *botF->rigidBody, btVector3(0, 0, pivot), btVector3(0, 0, pivot), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	//bot_botB
	join = new btHingeConstraint(*bot->rigidBody, *botB->rigidBody, btVector3(0, 0, -pivot), btVector3(0, 0, pivot), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	//botR_midR
	join = new btHingeConstraint(*botR->rigidBody, *midR->rigidBody, btVector3(-pivot, 0, 0), btVector3(0, -pivot, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//botL_midL
	join = new btHingeConstraint(*botL->rigidBody, *midL->rigidBody, btVector3(-pivot, 0, 0), btVector3(0, -pivot, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//botB_midB
	join = new btHingeConstraint(*botB->rigidBody, *midB->rigidBody, btVector3(0, 0, -pivot), btVector3(0, -pivot, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	//botF_midF
	join = new btHingeConstraint(*botF->rigidBody, *midF->rigidBody, btVector3(0, 0, -pivot), btVector3(0, -pivot, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	//topB_midB
	join = new btHingeConstraint(*topB->rigidBody, *midB->rigidBody, btVector3(0, 0, pivot), btVector3(0, pivot, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	//topF_midF
	join = new btHingeConstraint(*topF->rigidBody, *midF->rigidBody, btVector3(0, 0, pivot), btVector3(0, pivot, 0), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	//topL_midL
	join = new btHingeConstraint(*topL->rigidBody, *midL->rigidBody, btVector3(pivot, 0, 0), btVector3(0, pivot, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//topR_midR
	join = new btHingeConstraint(*topR->rigidBody, *midR->rigidBody, btVector3(pivot, 0, 0), btVector3(0, pivot, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//top_topL
	join = new btHingeConstraint(*top->rigidBody, *topL->rigidBody, btVector3(-pivot, 0, 0), btVector3(-pivot, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//top_topR
	join = new btHingeConstraint(*top->rigidBody, *topR->rigidBody, btVector3(pivot, 0, 0), btVector3(-pivot, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, 1), true);
	dynamicsWorld->addConstraint(join);

	//top_topF
	join = new btHingeConstraint(*top->rigidBody, *topF->rigidBody, btVector3(0, 0, pivot), btVector3(0, 0, -pivot), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	//top_topB
	join = new btHingeConstraint(*top->rigidBody, *topB->rigidBody, btVector3(0, 0, -pivot), btVector3(0, 0, -pivot), btVector3(1, 0, 0), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(join);

	return core;
}