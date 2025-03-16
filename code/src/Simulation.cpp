/*
WHISKiT Physics Simulator
Copyright (C) 2019 Nadina Zweifel (SeNSE Lab)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

This code is based on code published by
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

*/


#include "Simulation.h"

btVector4 BLUE = btVector4(0.,0.,1.0,1);
btVector4 RED = btVector4(1.,0.,0.0,1);
btVector4 GREEN = btVector4(0.,1.,0.0,1);
btVector4 GREY = btVector4(0.,0.,0.0,0.5);
btVector4 YELLOW = btVector4(1.,1.,0.0,1);
btVector4 ORANGE = btVector4(1.,0.647,0.0,1);


int Simulation::init_socket() {
    // Create socket
    clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1) {
        std::cerr << "Error creating socket" << std::endl;
        return 1;
    }

    // Configure server address
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SOCKET_PORT);

    // Convert IP address from string to network format
    if (inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr) <= 0) {
        std::cerr << "Invalid address/Address not supported" << std::endl;
        return 1;
    }

    // Connect to the server
    if (connect(clientSocket, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) == -1) {
        std::cerr << "Error connecting to server" << std::endl;
        return 1;
    }

	return 0;
}

void Simulation::load_parameters(Parameters& parameters) {
	TIME_STEP = parameters["TIME_STEP"].as< float >();
	NUM_STEP_INT = parameters["NUM_STEP_INT"].as< int >();
	NUM_STEP_SOLVER = parameters["NUM_STEP_SOLVER"].as< int >();
	TIME_STOP = parameters["TIME_STOP"].as< float >();
	NO_WHISKERS = parameters["NO_WHISKERS"].as< bool >();
	SAVE = parameters["SAVE"].as< bool >();
	ACTIVE = parameters["ACTIVE"].as< bool >();
	EXPLORING = parameters["EXPLORING"].as< bool >();
	DEBUG = parameters["DEBUG"].as< int >();
	PRINT = parameters["PRINT"].as< int >();

	camPos = parameters["CPOS"].as< std::vector< float > >();
	camDist = parameters["CDIST"].as< float >();
	camPitch = parameters["CPITCH"].as< float >();
	camYaw = parameters["CYAW"].as< float >();

	file_whisking_angle = parameters["file_whisking_angle"].as< std::string >();
	WHISKER_PARAMS_DIR = parameters["WHISKER_PARAMS_DIR"].as< std::string >();
	dir_rathead_trajectory = parameters["dir_rathead_trajectory"].as< std::string >();

	gravity = parameters["gravity"].as< std::vector< float > >();

	OBJECT = parameters["OBJECT"].as< int >();
	PEG_LOC = parameters["PEG_LOC"].as< std::vector< float > >();
	PEG_SPEED = parameters["PEG_SPEED"].as< float >();
	file_env = parameters["file_env"].as< std::string >();

	// get whiskers to simulate
	std::vector < std::string > whisker_names = parameters["WHISKER_NAMES"].as< std::vector< std::string > >();

	if (whisker_names[0] == "ALL") {
		whisker_names = {
			"LA0","LA1","LA2","LA3","LA4",
			"LB0","LB1","LB2","LB3","LB4",
			"LC0","LC1","LC2","LC3","LC4","LC5",
			"LD0","LD1","LD2","LD3","LD4","LD5",
			"LE1","LE2","LE3","LE4","LE5",
			"RA0","RA1","RA2","RA3","RA4",
			"RB0","RB1","RB2","RB3","RB4",
			"RC0","RC1","RC2","RC3","RC4","RC5",
			"RD0","RD1","RD2","RD3","RD4","RD5",
			"RE1","RE2","RE3","RE4","RE5"};
	}
	else if (whisker_names[0] == "R") {
		whisker_names = {
			"RA0","RA1","RA2","RA3","RA4",
			"RB0","RB1","RB2","RB3","RB4",
			"RC0","RC1","RC2","RC3","RC4","RC5",
			"RD0","RD1","RD2","RD3","RD4","RD5",
			"RE1","RE2","RE3","RE4","RE5"};
	}
	else if (whisker_names[0] == "L") {
		whisker_names = {
			"LA0","LA1","LA2","LA3","LA4",
			"LB0","LB1","LB2","LB3","LB4",
			"LC0","LC1","LC2","LC3","LC4","LC5",
			"LD0","LD1","LD2","LD3","LD4","LD5",
			"LE1","LE2","LE3","LE4","LE5"};
	}

	std::cout << "Whiskers to simulate: ";
	for (std::string s : whisker_names) {
		std::cout << s << " ";
	}
	std::cout << std::endl;

	// create rat
	rat = new Rat(m_guiHelper, &m_collisionShapes, whisker_names, parameters);
	btVector3 rathead_pos = rat->getPosition();

	data_dump->init(whisker_names);

	// set camera position to rat head
	camPos[0] = rathead_pos[0]+camPos[0];
	camPos[1] = rathead_pos[1]+camPos[1];
	camPos[2] = rathead_pos[2]+camPos[2];

	if(OBJECT==3){
		// create object from 3D scan
		btTransform xfrm;
		xfrm.setIdentity();
		btVector4 envColor = btVector4(0.6,0.6,0.6,1);
		env = new Object(m_guiHelper,&m_collisionShapes,xfrm,file_env,envColor,btScalar(SCALE),btScalar(0),COL_ENV,envCollidesWith);
	}
}

void Simulation::stepSimulation(){

	auto start = std::chrono::high_resolution_clock::now();
	m_time += TIME_STEP; 								// increase time
	m_step += 1;													// increase step

	// run simulation as long as stop time not exceeded
	if(TIME_STOP == 0 || m_time < TIME_STOP) {

		// register collisions
		rat->detect_collision(m_dynamicsWorld);


		// wait for "muscle control" from neural network script -- this is a blocking operation (wait until packet is received)
                // char buffer[1024];
		char buffer[20048];
		memset(buffer, 0, sizeof(buffer));
		recv(clientSocket, buffer, sizeof(buffer), 0);
		json recv_data = json::parse(buffer);
		std::cout << "Received data from neural network script " << std::endl;
		json parsed_data;
		std::string json_data(buffer);
		double x_velocity, y_velocity, z_velocity;
		double x_rotational_velocity, y_rotational_velocity, z_rotational_velocity;
		bool whisking;
		try
		{
			parsed_data = json::parse(json_data);
			x_velocity = parsed_data["x_velocity"];
			y_velocity = parsed_data["y_velocity"];
			z_velocity = parsed_data["z_velocity"];
			x_rotational_velocity = parsed_data["x_rotational_velocity"];
			y_rotational_velocity = parsed_data["y_rotational_velocity"];
			z_rotational_velocity = parsed_data["z_rotational_velocity"];
			whisking = parsed_data["whisking"];
		}
		catch (const json::parse_error& e)
		{
			std::cerr << "JSON PARSING ERROR: " << e.what() << std::endl;
		}
		catch (const json::type_error& e)
		{
			std::cerr << "JSON TYPE ERROR: " << e.what() << std::endl;
		}
		// decoding data received and do something with it -- for later
	
		json data_for_all_whiskers;
		for (size_t i = 0; i < rat->getWhiskerArraySize(); ++i) {
			const std::string w_name = rat->getWhisker(i)->getWhiskerName();
			const btVector3 force_from_whisker = rat->getWhisker(i)->getForces();
			const btVector3 torque_from_whisker = rat->getWhisker(i)->getTorques();
			const std::vector<int> collision = rat->get_C(i);


			json force = json::array();
			force.push_back(force_from_whisker[0]);
			force.push_back(force_from_whisker[1]);
			force.push_back(force_from_whisker[2]);
			json torque = json::array();
			torque.push_back(torque_from_whisker[0]);
			torque.push_back(torque_from_whisker[1]);
			torque.push_back(torque_from_whisker[2]);

			std::vector<std::vector<float>> positions;
			rat->getWhiskerLinkPositions(positions, i);

			json orientation = json::array();
			const btQuaternion orient = rat->getWhiskerOrientation(i);
			orientation.push_back(orient[0]);
			orientation.push_back(orient[1]);
			orientation.push_back(orient[2]);
			orientation.push_back(orient[3]);

			json whisker_data;
			whisker_data["X"] = positions[0];
			whisker_data["Y"] = positions[1];
			whisker_data["Z"] = positions[2];
			whisker_data["force"] = force;
			whisker_data["torque"] = torque;
			whisker_data["Collision"] = collision;
			whisker_data["Orientation"] = orientation;
			data_for_all_whiskers[w_name] = whisker_data;
		}

		json data;
		data["time"] = m_time,
		data["whiskers"] = data_for_all_whiskers;

		btVector3 ratPosition = rat->getPosition();
		json position_data = json::array();
		position_data.push_back(ratPosition[0]);
		position_data.push_back(ratPosition[1]);
		position_data.push_back(ratPosition[2]);
		data["Position"] = position_data;

		btVector3 ratEulerAngles = rat->getRotation();
		json angle_data = json::array();
		angle_data.push_back(ratEulerAngles[0]);
		angle_data.push_back(ratEulerAngles[1]);
		angle_data.push_back(ratEulerAngles[2]);
		data["Angle"] = angle_data;

		const btQuaternion& ratOrientation = rat->getOrientation();
		json orientation = json::array();
		orientation.push_back(ratOrientation[0]);
		orientation.push_back(ratOrientation[1]);
		orientation.push_back(ratOrientation[2]);
		orientation.push_back(ratOrientation[3]);
		data["Orientation"] = orientation;

		// std::cout << "Sending message: " << data << "...\n";
		std::string jsonData = data.dump();
		send(clientSocket, jsonData.c_str(), jsonData.size(), 0);

		// first, push back data into data_dump
		if(!NO_WHISKERS && SAVE) {
			rat->dump_M(data_dump);
			rat->dump_F(data_dump);
			rat->dump_Q(data_dump);
		}

		// moving object 1
		if(OBJECT==1){
			if(PEG_SPEED > 0){
				// btVector3 velocity = PEG_SPEED * btVector3(0.4,-1,0).normalized();
				btVector3 velocity = {0,0,0};
				peg->setLinearVelocity(velocity);
			}
		}

		// move array if in ACTIVE mode

		// move rat head if in EXPLORING mode
		if(EXPLORING){
			this_loc_vel = HEAD_LOC_VEL[m_step-1];
			rat->setLinearVelocity(btVector3(x_velocity, y_velocity, z_velocity));
			// rat->setLinearVelocity(btVector3(0, 0, 0));
			rat->setAngularVelocity(btVector3(x_rotational_velocity, y_rotational_velocity, z_rotational_velocity));
			// rat->setAngularVelocity(btVector3(0, 0, 0));
		}
		if(ACTIVE && !NO_WHISKERS && whisking){
			rat->whisk(recv_data["active_whisking_data"]);
		}

		// step simulation
		m_dynamicsWorld->stepSimulation(TIME_STEP,
		                                NUM_STEP_INT,
										TIME_STEP / NUM_STEP_INT);

		// draw debug if enabled
	    if(DEBUG) {
	    	m_dynamicsWorld->debugDrawWorld();
	    }

	    // set exit flag to zero
	    exitSim = 0;
	}
	else{
		// timeout -> set exit flg
		exitSim = 1;
	}

	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
	m_time_elapsed += duration.count()/1000.f;
	auto factor = m_time_elapsed / m_time;
	auto time_remaining = (int)((TIME_STOP - m_time) * (factor));
	if(PRINT==2){
		std::cout << "\rSimulation time: " << std::setprecision(2) << m_time << "s\tCompleted: " << std::setprecision(2) << m_time/TIME_STOP*100 << " %\tTime remaining: " << std::setprecision(4) << time_remaining/60 << " min " << std::setprecision(4) << (time_remaining % 60) << " s\n" << std::flush;
	}
}

void Simulation::initPhysics()
{
	// set visual axis
	m_guiHelper->setUpAxis(2);

	// create empty dynamics world[0]
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    // broadphase algorithm
    m_broadphase = new btDbvtBroadphase();

	// select solver
	std::cout << "Using btSequentialImpulseConstraintSolver..." << std::endl;
	m_solver = new btSequentialImpulseConstraintSolver();

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	// set number of iterations
	m_dynamicsWorld->getSolverInfo().m_numIterations = NUM_STEP_SOLVER;
	m_dynamicsWorld->getSolverInfo().m_solverMode = SOLVER_SIMD |
                        SOLVER_USE_WARMSTARTING |
                        SOLVER_RANDMIZE_ORDER |
                        0;
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;
	m_dynamicsWorld->getSolverInfo().m_erp = 0.8f;

	if (rat) {
		rat->initPhysics(m_dynamicsWorld);
	}

	// set gravity
	m_dynamicsWorld->setGravity(SCALE * btVector3(gravity[0], gravity[1], gravity[2]));

    // create debug drawer
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer()){
		if(DEBUG==1){
			std::cout << "DEBUG option 1: wireframes." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
		}
		else if(DEBUG==2){
			std::cout << "DEBUG option 2: constraints." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints);
		}
		else if(DEBUG==3){
			std::cout << "DEBUG option 3: wireframes & constraints." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawConstraintLimits);
		}
		else if(DEBUG==4){
			std::cout << "DEBUG option 4: AAbb." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawAabb);
		}
		else if(DEBUG==5){
			std::cout << "DEBUG option 5: Frammes." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawFrames);
		}
		else if(DEBUG==6){
			std::cout << "DEBUG option 6: Only collision" << std::endl;
			// m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawFrames);
		}
		else{
			std::cout << "No DEBUG." << std::endl;
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_NoDebug);
		}
	}

	// create object to collide with
	// peg
	if(OBJECT==1){
		btCollisionShape* pegShape = new btCylinderShapeZ(btVector3(1,1,80));
		pegShape->setMargin(0.1);
		m_collisionShapes.push_back(pegShape);
		btTransform trans = createFrame(btVector3(PEG_LOC[0], PEG_LOC[1], PEG_LOC[2]),btVector3(0, 0, 0));
		peg = createDynamicBody(1,0.5,trans, pegShape, m_guiHelper,  BLUE);
		m_dynamicsWorld->addRigidBody(peg,COL_ENV,envCollidesWith);
		peg->setActivationState(DISABLE_DEACTIVATION);
	}
	// create object to collide with wall
	else if(OBJECT==2){
		btCollisionShape* wallShape = new btBoxShape(btVector3(5,200,60));
		wallShape->setMargin(0.1);
		m_collisionShapes.push_back(wallShape);
		btTransform trans = createFrame(btVector3(50,0,0),btVector3(0,0,PI/6));
		wall = createDynamicBody(0,0.5, trans, wallShape, m_guiHelper,  BLUE);
		m_dynamicsWorld->addRigidBody(wall,COL_ENV,envCollidesWith);
	}
	else if(OBJECT==3){
		env->initPhysics(m_dynamicsWorld);
	}

	// generate graphics
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);

	resetCamera();

	// if active whisking, load whisking protraction angle trajectory
	if (ACTIVE) {
		const std::string dir_param = WHISKER_PARAMS_DIR;
		std::cout << "Reading active whisking data from " << dir_param + file_whisking_angle << "\n";

		read_csv_float(dir_param + file_whisking_angle, whisker_vel);

		TIME_STOP = std::min(TIME_STOP, (whisker_vel[0].size()/3 - 1) * TIME_STEP);
	}

	// if exploring, load data for rat head trajectory
	if (EXPLORING){
		read_csv_float(dir_rathead_trajectory, HEAD_LOC_VEL);
	}

	// initialize time/step tracker
	m_time_elapsed = 0;
	m_time = 0;
	m_step = 0;

	std::cout << "\n\nStart simulation..." << std::endl;
	std::cout << "\n====================================================\n" << std::endl;
}

output* Simulation::get_results(){
	return data_dump;
}

void Simulation::resetCamera(){
	m_guiHelper->resetCamera(camDist,camYaw,camPitch,camPos[0],camPos[1],camPos[2]);
}
