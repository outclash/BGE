#pragma once
#include "Game.h"
#include "GameComponent.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

using namespace std;

namespace BGE
{
	class Assignment :
		public Game
	{
	//private:

	public:
		Assignment(void);
		~Assignment(void);
		bool Initialise();
		void Update(float timeDelta);
		void Cleanup();

		//Machine 1
		shared_ptr<PhysicsController> M1;
		shared_ptr<PhysicsController> CreateM1();
		//procedural tower like 
		void CreateProcedural();
		int sR = 0;
	};
}