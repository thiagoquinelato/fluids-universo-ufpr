/*
  ======================================================================
   demo.c --- protoype to show off the simple solver
  ----------------------------------------------------------------------
   Author : Jos Stam (jstam@aw.sgi.com)
   Creation Date : Jan 9 2003

   Description:

	This code is a simple prototype that demonstrates how to use the
	code provided in my GDC2003 paper entitles "Real-Time Fluid Dynamics
	for Games". This code uses OpenGL and GLUT for graphics and interface

  =======================================================================
*/

#include <stdlib.h>
#include <stdio.h>
#include <GL/glut.h>
#include <stdbool.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

/* macros */

#define IX(i,j) ((i)+(N+2)*(j))

/* external definitions (from solver.c) */

extern void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt );
extern void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt );

/* global variables */

static int N;
static float dt, diff, visc;
static float force, source;
static int dvel;

static float * u, * v, * u_prev, * v_prev;
static float * dens, * dens_prev;
static float *dens_bloom, *ping, *pong;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int omx, omy, mx, my;


/*
  ----------------------------------------------------------------------
   free/clear/allocate simulation data
  ----------------------------------------------------------------------
*/


static void free_data ( void )
{
	if ( u ) free ( u );
	if ( v ) free ( v );
	if ( u_prev ) free ( u_prev );
	if ( v_prev ) free ( v_prev );
	if ( dens ) free ( dens );
	if ( dens_prev ) free ( dens_prev );

	if ( dens_bloom ) free ( dens_bloom );
	if ( ping ) free ( ping );
	if ( pong ) free ( pong );
}

float get_density_from_color(unsigned char r, unsigned char g, unsigned char b) {
        if (r > 200 && g > 200 && b > 200) return 1.0f;
        return 0.0f;
}

static void clear_data ( void )
{
	int i, size=(N+2)*(N+2);

	for ( i=0 ; i<size ; i++ ) {
		u[i] = v[i] = u_prev[i] = v_prev[i] = dens[i] = dens_prev[i] = 0.0f;
	}

	// draw image into density map (recarrega imagem toda vez!!)
	int w, h, n;
	unsigned char *img = stbi_load("logo_matind.jpg", &w, &h, &n, 3);
	if (img == NULL) {
		printf("Failure reason: %s\n", stbi_failure_reason());
		exit(1);
	}

	int max_size = N + 2;
	for (int i = 1; i < max_size-1; ++i) {
		for (int j = 1; j < max_size-1; ++j) {
			int img_i = (i * h) / max_size;
			int img_j = (j * w) / max_size;
			int img_idx = (img_i * w + img_j) * 3;

			dens[IX(j, max_size - i - 1)] = get_density_from_color(img[img_idx + 0], img[img_idx + 1], img[img_idx + 2]);
		}
	}

	stbi_image_free(img);
}

static int allocate_data ( void )
{
	int size = (N+2)*(N+2);

	u			= (float *) malloc ( size*sizeof(float) );
	v			= (float *) malloc ( size*sizeof(float) );
	u_prev		= (float *) malloc ( size*sizeof(float) );
	v_prev		= (float *) malloc ( size*sizeof(float) );
	dens		= (float *) malloc ( size*sizeof(float) );
	dens_prev	= (float *) malloc ( size*sizeof(float) );

	dens_bloom  = (float *) malloc ( size*sizeof(float) );	
	ping  = (float *) malloc ( size*sizeof(float) );	
	pong  = (float *) malloc ( size*sizeof(float) );

	if ( !u || !v || !u_prev || !v_prev || !dens || !dens_prev || !dens_bloom || !ping || !pong ) {
		fprintf ( stderr, "cannot allocate data\n" );
		return ( 0 );
	}

	return ( 1 );
}


/*
  ----------------------------------------------------------------------
   OpenGL specific drawing routines
  ----------------------------------------------------------------------
*/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluOrtho2D ( 0.0, 1.0, 0.0, 1.0 );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
}

static void post_display ( void )
{
	glutSwapBuffers ();
}

static void draw_velocity ( void )
{
	int i, j;
	float x, y, h;

	h = 1.0f/N;

	glColor3f ( 1.0f, 1.0f, 1.0f );
	glLineWidth ( 1.0f );

	glBegin ( GL_LINES );

		for ( i=1 ; i<=N ; i++ ) {
			x = (i-0.5f)*h;
			for ( j=1 ; j<=N ; j++ ) {
				y = (j-0.5f)*h;

				glVertex2f ( x, y );
				glVertex2f ( x+u[IX(i,j)], y+v[IX(i,j)] );
			}
		}

	glEnd ();
}

float lerp( float a, float b, float t )
{
	return a + t*(b-a);
}

void set_color_from_density(float density) {
	if (density > 1.0f) density = 1.0f;
	glColor3f(lerp(0.68f, 1.0f, density), lerp(0.13f, 1.0f, density), lerp(0.14f, 1.0f, density));
}




int clamp(int v, int a, int b) {
	return v < a ? a : (v > b ? b : v);
}

void blur_pass(float* src, float* dest, bool horizontal) {

	float weight[] = { 0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216 };

	for (int i = 1; i <= N; ++i) {
		for (int j = 1; j <= N; ++j) {
			float result = src[IX(i, j)] * weight[0];
			for (int k = 1; k < 5; ++k) {
				if (horizontal) {
					result += src[IX(clamp(i + k, 1, N), j)] * weight[k];
					result += src[IX(clamp(i - k, 1, N), j)] * weight[k];
				} else {
					result += src[IX(i, clamp(j + k, 1, N))] * weight[k];
					result += src[IX(i, clamp(j - k, 1, N))] * weight[k];
				}
			}

			dest[IX(i, j)] = result;
		}
	}
}

void bloom(float threshold, int passes, float exposure, float gamma) {
	for (int i = 1; i <= N; ++i) {
		for (int j = 1; j <= N; ++j) {
			ping[IX(i, j)] = dens[IX(i, j)] > threshold ? dens[IX(i, j)] : 0.0f;
		}
	}

	for (int pass = 0; pass < passes; ++pass) {
		bool horizontal = (pass & 1) == 0;
		blur_pass(horizontal ? ping : pong, horizontal ? pong : ping, horizontal);
	}

	for (int i = 1; i <= N; ++i) {
		for (int j = 1; j <= N; ++j) {
			if ((passes & 1) == 0) {
				dens_bloom[IX(i, j)] = dens[IX(i, j)] + ping[IX(i, j)];
			} else {
				dens_bloom[IX(i, j)] = dens[IX(i, j)] + pong[IX(i, j)];
			}

			dens_bloom[IX(i, j)] = powf(1.0f - expf(-dens_bloom[IX(i, j)] * exposure), 1.0f / gamma);
		}
	}
}





static void draw_density ( void )
{

	bloom(0.9f, 10, 1.2f, 1.3f);

	int i, j;
	float x, y, h, d00, d01, d10, d11;

	h = 1.0f/N;

	glBegin ( GL_QUADS );

		for ( i=0 ; i<=N ; i++ ) {
			x = (i-0.5f)*h;
			for ( j=0 ; j<=N ; j++ ) {
				y = (j-0.5f)*h;

				d00 = dens_bloom[IX(i,j)];
				d01 = dens_bloom[IX(i,j+1)];
				d10 = dens_bloom[IX(i+1,j)];
				d11 = dens_bloom[IX(i+1,j+1)];

				// bilinear filtering
				// d00 = d01 = d10 = d11 = lerp(lerp(d00, d01, 0.5f), lerp(d10, d11, 0.5f), 0.5f);

				set_color_from_density(d00); glVertex2f ( x, y );
				set_color_from_density(d10); glVertex2f ( x+h, y );
				set_color_from_density(d11); glVertex2f ( x+h, y+h );
				set_color_from_density(d01); glVertex2f ( x, y+h );
			}
		}

	glEnd ();
}

/*
  ----------------------------------------------------------------------
   relates mouse movements to forces sources
  ----------------------------------------------------------------------
*/

static void get_from_UI ( float * d, float * u, float * v )
{
	int i, j, size = (N+2)*(N+2);

	for ( i=0 ; i<size ; i++ ) {
		u[i] = v[i] = d[i] = 0.0f;
	}

	if ( !mouse_down[0] && !mouse_down[2] ) return;

	i = (int)((       mx /(float)win_x)*N+1);
	j = (int)(((win_y-my)/(float)win_y)*N+1);

	if ( i<1 || i>N || j<1 || j>N ) return;

	if ( mouse_down[0] ) {
		u[IX(i,j)] = force * (mx-omx);
		v[IX(i,j)] = force * (omy-my);
	}

	if ( mouse_down[2] ) {
		d[IX(i,j)] = source;
	}

	omx = mx;
	omy = my;

	return;
}

/*
  ----------------------------------------------------------------------
   GLUT callback routines
  ----------------------------------------------------------------------
*/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
		case 'c':
		case 'C':
			clear_data ();
			break;

		case 'q':
		case 'Q':
			free_data ();
			exit ( 0 );
			break;

		case 'v':
		case 'V':
			dvel = !dvel;
			break;
	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omx = my = y;

	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

static void idle_func ( void )
{
	get_from_UI ( dens_prev, u_prev, v_prev );
	vel_step ( N, u, v, u_prev, v_prev, visc, dt );
	dens_step ( N, dens, dens_prev, u, v, diff, dt );

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{
	pre_display ();

		if ( dvel ) draw_velocity ();
		else		draw_density ();

	post_display ();
}


/*
  ----------------------------------------------------------------------
   open_glut_window --- open a glut compatible window and set callbacks
  ----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
	glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE );

	glutInitWindowPosition ( 0, 0 );
	glutInitWindowSize ( win_x, win_y );
	win_id = glutCreateWindow ( "Alias | wavefront" );

	glClearColor ( 1.0f, 1.0f, 1.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();
	glClear ( GL_COLOR_BUFFER_BIT );
	glutSwapBuffers ();

	pre_display ();

	glutKeyboardFunc ( key_func );
	glutMouseFunc ( mouse_func );
	glutMotionFunc ( motion_func );
	glutReshapeFunc ( reshape_func );
	glutIdleFunc ( idle_func );
	glutDisplayFunc ( display_func );
}


/*
  ----------------------------------------------------------------------
   main --- main routine
  ----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	if ( argc != 1 && argc != 7 ) {
		fprintf ( stderr, "usage : %s N dt diff visc force source\n", argv[0] );
		fprintf ( stderr, "where:\n" );\
		fprintf ( stderr, "\t N      : grid resolution\n" );
		fprintf ( stderr, "\t dt     : time step\n" );
		fprintf ( stderr, "\t diff   : diffusion rate of the density\n" );
		fprintf ( stderr, "\t visc   : viscosity of the fluid\n" );
		fprintf ( stderr, "\t force  : scales the mouse movement that generate a force\n" );
		fprintf ( stderr, "\t source : amount of density that will be deposited\n" );
		exit ( 1 );
	}

	if ( argc == 1 ) {
		N = 64;
		dt = 0.1f;
		diff = 0.0f;
		visc = 0.0f;
		force = 5.0f;
		source = 100.0f;
		fprintf ( stderr, "Using defaults : N=%d dt=%g diff=%g visc=%g force = %g source=%g\n",
			N, dt, diff, visc, force, source );
	} else {
		N = atoi(argv[1]);
		dt = atof(argv[2]);
		diff = atof(argv[3]);
		visc = atof(argv[4]);
		force = atof(argv[5]);
		source = atof(argv[6]);
	}

	printf ( "\n\nHow to use this demo:\n\n" );
	printf ( "\t Add densities with the right mouse button\n" );
	printf ( "\t Add velocities with the left mouse button and dragging the mouse\n" );
	printf ( "\t Toggle density/velocity display with the 'v' key\n" );
	printf ( "\t Clear the simulation by pressing the 'c' key\n" );
	printf ( "\t Quit by pressing the 'q' key\n" );

	dvel = 0;

	if ( !allocate_data () ) exit ( 1 );
	clear_data ();


	win_x = 512;
	win_y = 512;
	open_glut_window ();

	glutMainLoop ();

	exit ( 0 );
}
