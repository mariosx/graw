#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <gl\glfw.h>
#include <math.h>

//class three_float?
class Color
{	public:
		float r, g, b;
		void operator= (Color initial)
		{	this->r = initial.r;
			this->g = initial.g;
			this->b = initial.b;
			return; }
};
class Rot_Vector
{	public:
		float ab_x, ab_y, ab_z;
		void operator= (Rot_Vector initial)
		{	this->ab_x = initial.ab_x;
			this->ab_y = initial.ab_y;
			this->ab_z = initial.ab_z;
			return; }
		void operator+= (Rot_Vector beta)
		{	this->ab_x += beta.ab_x;
			this->ab_y += beta.ab_y;
			this->ab_z += beta.ab_z;
			return; }
};
class Euler_Angle { public: float theta, phi; };//faster than axis-based rot_vectors?
class Vector3//could make this basically a linklist so as to easily step through the base vectors to find "real vector"
	//for instance, have (the relationship of a vertex to it's obj) as one vector with (the obj's relationship to the origin) as it's base vector,
	//when doing hard coordinate calculations, they could be added up through a function that returns the "pure" vector
	//would this idea make rotation calulations easier? (partial answer: would eliminate requirement for translation to origin before hard body rotation, internal origin used instead)
	//This could make newtonian-system (think solar or planetary system) branching easier as well
	//would certainly require a formal rotation matrix function (still need to find one that isnt for Euler angles)
	//a vector class variable might be a good idea as well (think coordinate vs velocity vs acceleration)
{	protected:
		float x, y, z;
	public:
		float getx (void) {	return x; }
		float gety (void) {	return y; }
		float getz (void) {	return z; }
		void init (float x, float y, float z)
		{	this->x = x; this->y = y; this->z = z; }	
		void operator= (Vector3 initial)
		{	this->init( initial.getx(), initial.gety(), initial.getz() ); return; }
		int operator== (Vector3 b)
		{	return b.getx() == this->getx() && b.gety() == this->gety() && b.getz() == this->getz(); }
		void operator+= (Vector3 delta)
		{	this->init( this->getx() + delta.getx(),
						this->gety() + delta.gety(),
						this->getz() + delta.getz() );
			return; }
		Vector3 operator+ (Vector3 beta)
		{	Vector3 gamma;
			gamma.init ( beta.x + this->x, beta.y + this->y, beta.z + this->z );
			return gamma; }
		Vector3 operator- (Vector3 beta)
		{	Vector3 gamma;
			gamma.init ( this->x - beta.x, this->y - beta.y, this->z - beta.z );
			return gamma; }
		Vector3 operator* (float scalar)
		{	Vector3 gamma;
			gamma.init ( scalar * this->x, scalar * this->y, scalar * this->z );
			return gamma; }
		Vector3 cross_product (Vector3 b)
		{	Vector3 c;
			c.init ( this->gety() * b.getz() - this->getz() * b.gety(),
					 this->getz() * b.getx() - this->getx() * b.getz(),
					 this->getx() * b.gety() - this->gety() * b.getx() );
			return c; }
		float dot_product (Vector3 b)
		{	float c;
			c = x * b.getx() + y * b.gety() + z * b.getz();
			return c; }
		void rotate_this (Rot_Vector spin, Vector3 rot_center)//~_this changes the contents, ~d returns transformed without changing contents
		{	*this += (rot_center * -1); //'this' transformed to be the coordinates of 'this', relative to rot_center
			this->init(this->getx(), this->y*cosf( spin.ab_x ) - this->z*sinf( spin.ab_x ), this->z*cosf( spin.ab_x ) + this->y*sinf( spin.ab_x ));// 2-D transformation of y and z (about the x axis)
			this->init(this->x*cosf( spin.ab_y ) - this->z*sinf( spin.ab_y ), this->gety(), this->z*cosf( spin.ab_y ) + this->x*sinf( spin.ab_y ));// changes x and z, about y
			this->init(this->x*cosf( spin.ab_z ) - this->y*sinf( spin.ab_z ), this->y*cosf( spin.ab_z ) + this->x*sinf( spin.ab_z ), this->getz());// changes x and y, about z
			*this += rot_center;
			return; }
		void orient_to (Vector3 center, Rot_Vector offset)
		{	*this = (*this + center * -1).rotated(offset, center); return; }
		Vector3 oriented_to (Vector3 center, Rot_Vector offset)
		{	return (*this + center * -1).rotated(offset, center); }
		Vector3 rotated (Rot_Vector spin, Vector3 rot_center)//are these functions mathematically correct?
		{	Vector3 f = *this; //f needs to be the coordinates of 'this', relative to rot_center
			f.init( f.getx(),
					f.y*cosf( spin.ab_x ) + f.z*sinf( spin.ab_x ),//unsure if the signs are correct
					f.z*cosf( spin.ab_x ) - f.y*sinf( spin.ab_x ));
					// 2-D transformation of y and z (about the x-axis)
			f.init( f.x*cosf( spin.ab_y ) + f.z*sinf( spin.ab_y ),
					f.gety(),
					f.z*cosf( spin.ab_y ) - f.x*sinf( spin.ab_y ));
					// changes x and z, about y-axis
			f.init( f.x*cosf( spin.ab_z ) + f.y*sinf( spin.ab_z ),
					f.y*cosf( spin.ab_z ) - f.x*sinf( spin.ab_z ),
					f.getz());// changes x and y, about z-axis
			f += rot_center;// by subtracting before and re-adding after, the rotation should be about the "new" origin, rot_center
			return f; }
};
class object// are there class prototypes?
{	public:
		Vector3 position;
		Vector3 velocity;
		Rot_Vector facing;
};
class Poly
{	public: Vector3 vertices[4]; Vector3 center; int num_v; Color color;
	Vector3 get_center (void) {	return center; }
	Color get_color (void) {	return color; }
	void rotate_this (Rot_Vector spin, Vector3 rot_center)
	{	for (int i = 0; i < num_v; i++)
		{	vertices[i].rotate_this( spin, rot_center ); }
		return; }
	void operator= (Poly init)
	{	this->num_v = init.num_v;
		for (int i = 0; i < init.num_v; i++)
			this->vertices[i] = init.vertices[i];
		this->color = init.color;
		this->center = init.center; }
};

const float alpha[3] = {0.0f,0.5f,1.0f};
const Color white = {1.0,1.0,1.0}, red =     {1.0,0.0,0.0}, green =  {0.0,1.0,0.0}, blue = {0.0,0.0,1.0},
			black = {0.0,0.0,0.0}, magenta = {1.0,0.0,1.0}, yellow = {1.0,1.0,0.0}, teal = {0.0,1.0,1.0};

void GLFWCALL handleResize(int w,int h)
{	glViewport( 0, 0, w, h );
	glMatrixMode( GL_PROJECTION );					//Switch to setting the camera perspective
	glLoadIdentity();								//reset the camera
	gluPerspective( 45, ( (float)w / (float)h ), 1, 100);
	return; }
void display (Poly to_draw[], int num_p, object camera)
{	glClearColor( white.r,white.g,white.b, 0.0f ); //clears background screen
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glMatrixMode( GL_MODELVIEW ); //switch to setting the draw frame
	for (int p = 0; p < num_p; p++)
	{	glColor3f( to_draw[p].color.r, to_draw[p].color.g, to_draw[p].color.b );
		glBegin(GL_POLYGON);
			for (int i = 0; i < to_draw[p].num_v; i++)
			{	glVertex3f( to_draw[p].vertices[i].oriented_to(camera.position, camera.facing).getx(), 
							to_draw[p].vertices[i].oriented_to(camera.position, camera.facing).gety(),
							to_draw[p].vertices[i].oriented_to(camera.position, camera.facing).getz() );
			}
		glEnd(); }
    glfwSwapBuffers();
	return; }
void control (object *camera)//currently movement rotation is correct, but not view rotation
{	Rot_Vector add_facing; add_facing.ab_x = 0; add_facing.ab_y = 0; add_facing.ab_z = 0; 
	if (glfwGetKey( GLFW_KEY_UP ) == GLFW_PRESS && glfwGetKey( GLFW_KEY_DOWN ) != GLFW_PRESS )
	{	add_facing.ab_x = 0.001f; }
	else if (glfwGetKey( GLFW_KEY_DOWN ) == GLFW_PRESS && glfwGetKey(GLFW_KEY_UP) != GLFW_PRESS)
	{	add_facing.ab_x = -0.001f; }
	if (glfwGetKey( GLFW_KEY_LEFT ) == GLFW_PRESS && glfwGetKey( GLFW_KEY_RIGHT ) != GLFW_PRESS)
	{	add_facing.ab_y = -0.001f; }
	else if (glfwGetKey( GLFW_KEY_RIGHT ) == GLFW_PRESS && glfwGetKey( GLFW_KEY_LEFT ) != GLFW_PRESS)
	{	add_facing.ab_y += 0.001f; }
	camera->facing += add_facing;
	Vector3 w,s,a,d;
	w.init(    0, 0,-0.02 );//signs might not be right, check later
	s.init(    0, 0, 0.02 );
	a.init(-0.02, 0, 0	  );
	d.init( 0.02, 0, 0	  );
	Vector3 null; null.init(0,0,0);
	Vector3 vi = camera->velocity;
	camera->velocity = null;
	if (glfwGetKey('W') == GLFW_PRESS && vi == null)
	{	camera->velocity += w; }//if rotated around camera position, would have to translate wasd vector first
	else if (glfwGetKey('S') == GLFW_PRESS && vi == null)
	{	camera->velocity += s; }//clearly currently acts as if this says '=' instead of '+='. Is '+=' not working????
	if (glfwGetKey('A') == GLFW_PRESS && vi == null)
	{	camera->velocity += a; }//yeah, really not working!!!
	else if (glfwGetKey('D') == GLFW_PRESS && vi == null)
	{	camera->velocity += d; }
	camera->position += camera->velocity.oriented_to(camera->position, camera->facing);//currently in rot, 
	return; }
int main (void)
{	int running = GL_TRUE, height = 0, width = 0;
	Poly to_draw[2];
	Vector3 center;
	object camera;
	
	camera.position.init(0,0,0);
	camera.facing.ab_x = 0;
	camera.facing.ab_y = 0;
	camera.facing.ab_z = 0;

	float sidelen = 1.0; int i = 0, v = 0;
	center.init(0,0,-5);
	to_draw[i].color = green;
	to_draw[i].num_v = 4;
	to_draw[i].center.init( 0, 0,-5 );
	to_draw[i].vertices[v++].init( center.getx() + sidelen, center.gety() + sidelen, center.getz() );
	to_draw[i].vertices[v++].init( center.getx() + sidelen, center.gety() - sidelen, center.getz() );
	to_draw[i].vertices[v++].init( center.getx() - sidelen, center.gety() - sidelen, center.getz() );
	to_draw[i].vertices[v  ].init( center.getx() - sidelen, center.gety() + sidelen, center.getz() );
	
	sidelen = 0.5; i = 1; v = 0; center.init(0,0,-4);
	to_draw[i].color = red;
	to_draw[i].num_v = 4;
	to_draw[i].center.init( 0, 0,-4 );
	to_draw[i].vertices[v++].init( center.getx() + sidelen,  center.gety() + sidelen, center.getz() );
	to_draw[i].vertices[v++].init( center.getx() + sidelen,  center.gety() - sidelen, center.getz() );
	to_draw[i].vertices[v++].init( center.getx() - sidelen,  center.gety() - sidelen, center.getz() );
	to_draw[i].vertices[v  ].init( center.getx() - sidelen,  center.gety() + sidelen, center.getz() );

	glfwInit();
	//Makes 3D drawing work when something is in front of something else
	glEnable(GL_DEPTH_TEST);
	if(!glfwOpenWindow(1920/1.5, 1080/1.5, 0,0,0,0,0,0, GLFW_WINDOW))
	{	glfwTerminate(); return 0; }
	glfwSetWindowSizeCallback(handleResize);
	while (running)
	{	control( &camera );
		display( to_draw, 2, camera );
		running = !glfwGetKey( GLFW_KEY_ESC ) && glfwGetWindowParam( GLFW_OPENED ); }
	glfwCloseWindow();
	glfwTerminate();
	return 0; }