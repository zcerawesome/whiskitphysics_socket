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
#include "Parameters.h"
#include "CommonInterfaces/CommonExampleInterface.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Quaternion.h"

#include "LinearMath/btTransform.h"
#include "LinearMath/btHashMap.h"
#include "Utils/b3Clock.h"
#include "ExampleBrowser/OpenGLGuiHelper.h"

#include <stdio.h>
#include <iostream>
#include <boost/program_options.hpp>

#include <signal.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>

#include <boost/lexical_cast.hpp>

using boost::lexical_cast;

volatile sig_atomic_t exitFlag = 0;

void exit_function(int sigint)
{
	exitFlag = 1;
}
template <typename Out>
void split(const std::string &s, char delim, Out result) {
    std::istringstream iss(s);
    std::string item;
    while (std::getline(iss, item, delim)) {
        *result++ = item;
    }
}
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

char* gVideoFileName = 0;
char* gPngFileName = 0;

static b3WheelCallback sOldWheelCB = 0;
static b3ResizeCallback sOldResizeCB = 0;
static b3MouseMoveCallback sOldMouseMoveCB = 0;
static b3MouseButtonCallback sOldMouseButtonCB = 0;
static b3KeyboardCallback sOldKeyboardCB = 0;
//static b3RenderCallback sOldRenderCB = 0;

float gWidth = 1024;
float gHeight = 768;

Simulation*    simulation;
int gSharedMemoryKey=-1;

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void OnMouseMove( float x, float y)
{
	bool handled = false;
	handled = simulation->mouseMoveCallback(x,y);

	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback (x,y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback  = 0;
static void OnMouseDown(int button, int state, float x, float y) {
	bool handled = false;

	handled = simulation->mouseButtonCallback(button, state, x,y);
	if (!handled)
	{
		if (prevMouseButtonCallback )
			prevMouseButtonCallback (button,state,x,y);
	}
}

class LessDummyGuiHelper : public DummyGUIHelper
{
	CommonGraphicsApp* m_app;
public:
	virtual CommonGraphicsApp* getAppInterface()
	{
		return m_app;
	}

	LessDummyGuiHelper(CommonGraphicsApp* app)
		:m_app(app)
	{
	}
};

int main(int argc, char** argv)
{
	signal(SIGINT, exit_function);

	std::cout.precision(17);

	std::string parameters_fn;

  	try
  	{ /** Define and parse the program options  */
		namespace po = boost::program_options;
		po::options_description desc("Options");
		desc.add_options()
		("help,h", "Help screen")
		("parameters", po::value<std::string>(&parameters_fn), "Filename or command-separated list of filenames, containing default parameters in YAML format");

		po::variables_map vm;

		try {
			po::store(po::parse_command_line(argc, argv, desc,po::command_line_style::unix_style ^ po::command_line_style::allow_short), vm); // can throw
		 	po::notify(vm);

		 	if ( vm.count("help")  ) {
				std::cout << "Bullet Whisker Simulation" << std::endl
						  << desc << std::endl;
				return 0;
			}
			std::cout << "ARG  = " << parameters_fn << "\n";
			Parameters parameters;
			for (std::string fn : split(parameters_fn, ',')) {
				std::cout << "Reading parameters from file: " << fn << std::endl;
  				parameters = mergeNodes(parameters, read_parameters_from_file(fn));
			}


			SimpleOpenGL3App* app = new SimpleOpenGL3App("Bullet Whisker Simulation",1024,768,true);

			prevMouseButtonCallback = app->m_window->getMouseButtonCallback();
			prevMouseMoveCallback = app->m_window->getMouseMoveCallback();

			app->m_window->setMouseButtonCallback((b3MouseButtonCallback)OnMouseDown);
			app->m_window->setMouseMoveCallback((b3MouseMoveCallback)OnMouseMove);

			OpenGLGuiHelper gui(app,false);
			CommonExampleOptions options(&gui);

			simulation = new Simulation(options.m_guiHelper);
			simulation->load_parameters(parameters);
			simulation->initPhysics();
			simulation->resetCamera();
			simulation->init_socket();
			char fileName[1024];
			int textureWidth = 128;
			int textureHeight = 128;

			unsigned char* image = new unsigned char[textureWidth*textureHeight * 4];
			int textureHandle = app->m_renderer->registerTexture(image, textureWidth, textureHeight);

			// int cubeIndex = app->registerCubeShape(1, 1, 1);

			// b3Vector3 pos = b3MakeVector3(0, 0, 0);
			// b3Quaternion orn(0, 0, 0, 1);
			// b3Vector3 color = b3MakeVector3(1, 0, 0);
			// b3Vector3 scaling = b3MakeVector3 (1, 1, 1);

			if(parameters["SAVE_VIDEO"].as< bool >()){
				std::string videoname = parameters["file_video"].as< std::string >();
				gVideoFileName = &videoname[0];

				if (gVideoFileName){
					std::cout << "Rendering video..." << std::endl;
					app->dumpFramesToVideo(gVideoFileName);

				}

				std::string pngname = "png_test";
				gPngFileName = &pngname[0];

				// app->m_renderer->registerGraphicsInstance(cubeIndex, pos, orn, color, scaling);
				app->m_renderer->writeTransforms();


			}

			do
			{
				if(parameters["SAVE_VIDEO"].as< bool >()){
					static int frameCount = 0;
					frameCount++;

					if (gPngFileName)
					{
						// printf("gPngFileName=%s\n", gPngFileName);

						sprintf(fileName, "%s%d.png", gPngFileName, frameCount++);
						app->dumpNextFrameToPng(fileName);
					}



					//update the texels of the texture using a simple pattern, animated using frame index
					for (int y = 0; y < textureHeight; ++y)
					{
						const int	t = (y + frameCount) >> 4;
						unsigned char*	pi = image + y*textureWidth * 3;
						for (int x = 0; x < textureWidth; ++x)
						{
							const int		s = x >> 4;
							const unsigned char	b = 180;
							unsigned char			c = b + ((s + (t & 1)) & 1)*(255 - b);
							pi[0] = pi[1] = pi[2] = pi[3] = c; pi += 3;
						}
					}

					app->m_renderer->activateTexture(textureHandle);
					app->m_renderer->updateTexture(textureHandle, image);

					float color[4] = { 255, 1, 1, 1 };
					app->m_primRenderer->drawTexturedRect(100, 200, gWidth / 2 - 50, gHeight / 2 - 50, color, 0, 0, 1, 1, true);
				}

				app->m_instancingRenderer->init();
		    	app->m_instancingRenderer->updateCamera(app->getUpAxis());

				simulation->stepSimulation();

				if (parameters["DEBUG"].as< int >() != 1) {
					simulation->renderScene();
					app->m_renderer->renderScene();
				}

				DrawGridData dg;
		        dg.upAxis = app->getUpAxis();
				app->setBackgroundColor(1,1,1);
				app->drawGrid(dg);
				// char bla[1024];
				// std::sprintf(bla, "Simple test frame %d", frameCount);
				// app->drawText(bla, 10, 10);
				app->swapBuffer();
			} while (!app->m_window->requestedExit() && !(exitFlag || simulation->exitSim));

			std::cout << "Saving data..." << std::endl;
			if (parameters["SAVE"].as< bool >()) {
				std::cout << "Simualtion terminated." << std::endl;
				output* results = simulation->get_results();
				const string output_dir = parameters["dir_out"].as< string >();
				save_data(results, output_dir);
			}

			std::cout << "Exit simulation..." << std::endl;
			simulation->exitPhysics();

			delete simulation;
			delete app;
			delete[] image;
			std::cout << "Done." << std::endl;

		}
		catch(po::error& e)
		{
		  std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
		  std::cerr << desc << std::endl;
		  return 1;
		}
  	}
  	catch(std::exception& e)
  	{
		std::cerr << "Unhandled Exception reached the top of main_opengl: " << e.what() << ", application will now exit" << std::endl;
		return 2;
  	}

	return 0;
}
