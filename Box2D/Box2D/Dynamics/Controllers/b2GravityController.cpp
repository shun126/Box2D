/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "b2GravityController.h"

void b2GravityController::Step(const b2TimeStep& step)
{
	B2_NOT_USED(step);
	if(invSqr){
		for(b2ControllerEdge *i=m_bodyList;i;i=i->nextBody){
			b2Body* body1 = i->body;
			for(b2ControllerEdge *j=m_bodyList;j!=i;j=j->nextBody){
				b2Body* body2 = j->body;
				b2Vec2 d = body1->GetWorldCenter() - body1->GetWorldCenter();
				float32 r2 = d.LengthSquared();
				b2Vec2 f = G / r2 / sqrt(r2) * body1->GetMass() * body2->GetMass() * d;
				body1->ApplyForce(f      , body1->GetWorldCenter(), true);
				body2->ApplyForce(-1.0f*f, body2->GetWorldCenter(), true);
			}
		}
	}else{
		for(b2ControllerEdge *i=m_bodyList;i;i=i->nextBody){
			b2Body* body1 = i->body;
			for(b2ControllerEdge *j=m_bodyList;j!=i;j=j->nextBody){
				b2Body* body2 = j->body;
				b2Vec2 d = body1->GetWorldCenter() - body1->GetWorldCenter();
				float32 r2 = d.LengthSquared();
				b2Vec2 f = G / r2 * body1->GetMass() * body2->GetMass() * d;
				body1->ApplyForce(f      , body1->GetWorldCenter(), true);
				body2->ApplyForce(-1.0f*f, body2->GetWorldCenter(), true);
			}
		}
	}

}