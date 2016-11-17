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

#ifndef BUOYANCY_H
#define BUOYANCY_H

class Buoyancy : public Test
{
public:
	Buoyancy()
	{
		b2BuoyancyController* bc = &m_bc;
		m_world->AddController(bc);

		bc->offset = 7.5f;
		bc->normal.Set(0,1);
		bc->density = 2;
		bc->linearDrag = 2;
		bc->angularDrag = 1;
		bc->useDensity = true;

		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			bd.position.Set(0.0f, -10.0f);
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
			ground->CreateFixture(&shape, 0.0f);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(0.5f, 0.125f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 2.0f;
			fd.friction = 0.2f;

			b2RevoluteJointDef jd;
			const int32 numPlanks = 30;

			b2Body* prevBody = ground;
			for (int32 i = 0; i < numPlanks; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-14.5f + 1.0f * i, 5.0f);
				b2Body* body = m_world->CreateBody(&bd);
				b2Fixture* fixture = body->CreateFixture(&fd);
				//body->ResetMassData();
				//body->SetMassFromShapes();

				b2Vec2 anchor(-15.0f + 1.0f * i, 5.0f);
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);

				prevBody = body;

				bc->AddBody(body);
			}

			b2Vec2 anchor(-15.0f + 1.0f * numPlanks, 5.0f);
			jd.Initialize(prevBody, ground, anchor);
			m_world->CreateJoint(&jd);
		}

		for (int32 i = 0; i < 3; ++i)
		{
			b2PolygonShape shape;
			shape.m_count = 3;
			shape.m_vertices[0].Set(-0.5f, 0.0f);
			shape.m_vertices[1].Set(0.5f, 0.0f);
			shape.m_vertices[2].Set(0.0f, 1.5f);

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-8.0f + 8.0f * i, 12.0f);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&shape, 1.0f);
			//body->ResetMassData();
			//body->SetMassFromShapes();

			bc->AddBody(body);
		}

		for (int32 i = 0; i < 3; ++i)
		{
			b2CircleShape shape;
			shape.m_radius = 0.5f;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(-6.0f + 6.0f * i, 10.0f);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&shape, 1.0f);
			//body->ResetMassData();
			//body->SetMassFromShapes();

			bc->AddBody(body);
		}
	}

	static Test* Create()
	{
		return new Buoyancy;
	}
private:
	b2BuoyancyController m_bc;
};

#endif
