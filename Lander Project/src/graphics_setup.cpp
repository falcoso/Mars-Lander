#include "graphics_setup.h"

void enable_lights(void)
// Enable the appropriate subset of lights
{
	if (static_lighting) {
		glDisable(GL_LIGHT0); glDisable(GL_LIGHT1);
		glEnable(GL_LIGHT2); glEnable(GL_LIGHT3);
		glDisable(GL_LIGHT4); glDisable(GL_LIGHT5);
	}
	else {
		glEnable(GL_LIGHT0); glEnable(GL_LIGHT1);
		glDisable(GL_LIGHT2); glDisable(GL_LIGHT3);
		glDisable(GL_LIGHT4); glDisable(GL_LIGHT5);
	}
}

void setup_lights(void)
// Specifies attributes of all lights, enables a subset of lights according to the lighting model
{
	GLfloat none[] = { 0.0, 0.0, 0.0, 1.0 };
	GLfloat low[] = { 0.15f, 0.15f, 0.15f, 1.0f };
	GLfloat medium[] = { 0.5, 0.5, 0.5, 1.0 };
	GLfloat high[] = { 0.75, 0.75, 0.75, 1.0 };

	// Lights 0 and 1 are for the dynamic lighting model, with the lights fixed in the viewer's reference frame
	glLightfv(GL_LIGHT0, GL_AMBIENT, none);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, high);
	glLightfv(GL_LIGHT0, GL_SPECULAR, none);
	glLightfv(GL_LIGHT0, GL_POSITION, top_right);
	glLightfv(GL_LIGHT1, GL_AMBIENT, none);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, medium);
	glLightfv(GL_LIGHT1, GL_SPECULAR, none);
	glLightfv(GL_LIGHT1, GL_POSITION, straight_on);

	// Lights 2 and 3 are for the static lighting model, with the lights fixed in the planetary reference frame
	glLightfv(GL_LIGHT2, GL_AMBIENT, none);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, high);
	glLightfv(GL_LIGHT2, GL_SPECULAR, none);
	glLightfv(GL_LIGHT3, GL_AMBIENT, low);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, none);
	glLightfv(GL_LIGHT3, GL_SPECULAR, none);

	// Lights 4 and 5 are for highlighting the lander with static lights, to avoid flat views with ambient illumination only
	glLightfv(GL_LIGHT4, GL_AMBIENT, none);
	glLightfv(GL_LIGHT4, GL_DIFFUSE, low);
	glLightfv(GL_LIGHT4, GL_SPECULAR, none);
	glLightfv(GL_LIGHT5, GL_AMBIENT, none);
	glLightfv(GL_LIGHT5, GL_DIFFUSE, low);
	glLightfv(GL_LIGHT5, GL_SPECULAR, none);

	enable_lights();
}