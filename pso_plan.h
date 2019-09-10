#ifndef PSO_PLAN_H
#define PSO_PLAN_H
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <vector>
using std::vector;

#define delta 0.300f
// #define float_eq(x, y) ( fabs((x) - (y)) < delta )
#define sq(x) ((x)*(x))

extern int debug_threshold_level;
extern bool dbg_lv_locked;
int level_debug(int level, char const*__restrict fmt, ...);

struct pso_conf_t
{
	float inertia_weight_max;
	float inertia_weight_min;
	float c1;
	float c2;
	int particle_amount;
	int dimension;
	int iteration_max;
//	char map_file[FILENAME_MAX];
};


struct vertex_t { float x, y; };
struct coefficient_t { float a, b, c; };
typedef struct
{ 
	int vert_cnt; 
	vertex_t * verts;
	coefficient_t * coefs; //Coefficient of obstacle's outlines 
	void print()
	{
		level_debug( 5, "obstacle: ");
		for( int i=0; i<vert_cnt; ++i)
			level_debug( 5, " (%.2f, %.2f)", verts[i].x, verts[i].y);
		level_debug( 5, "\n");
	}
} obstacle_t;

struct line_t
{
	vertex_t vert_s;
	vertex_t vert_e;
};

struct pso_map_t
{ 
	//include room and obstacle, treat room as obstacle can simplify my code
	int obst_cnt;  // actually obst_cnt = real_obstacle_cnt + 1
	obstacle_t *obsts;

};

struct path_t
{
	float * y_pos; //array
	float * velocity; //array
	float fitness;
	int dimension;
	bool isbad;
	void init(int dim, bool init_vel = true)
	{
		y_pos = new float[dim];
		assert( y_pos );
		if( init_vel )
		{
			velocity = new float[dim];
			assert( velocity );
		}
		else
			velocity = NULL;
		dimension = dim;
		fitness = 0;
		isbad = false;
	}
	void calc_fitness(double step)
	{
		fitness = 0;
		fitness += sqrtf(sq(step) + sq(y_pos[0]));
		for( int i=0; i<dimension-1; ++i)
			fitness += sqrtf(sq(step) + sq(y_pos[i+1]-y_pos[i]));
		fitness += sqrtf(sq(step) + sq(y_pos[dimension-1]));
		level_debug( 4, "%s: %f\n", __FUNCTION__, fitness);
	}
	void clone(path_t * p, bool clone_vel = false)
	{
		memcpy( y_pos, p->y_pos, dimension * sizeof(float));
		if( clone_vel )
			memcpy( velocity, p->velocity, dimension * sizeof(float));
		fitness = p->fitness;
		dimension = p->dimension;
	}
	void destroy()
	{
		delete []y_pos;
		delete []velocity;
		y_pos = NULL;
		velocity = NULL;
		dimension = 0;
		fitness = 0;
		isbad = false;
	}
};

extern vertex_t offset;  //offset for coordinate convert
extern float cosA, sinA;
void coord1_to_coord2( vertex_t * vert );
void coord2_to_coord1( vertex_t * vert );


typedef struct
{ 
	int vert_cnt; 
	vertex_t * verts;
	void init(const int cnt)
	{
		vert_cnt = cnt;
		verts = new vertex_t[cnt];
		assert(verts);
	}
	void clone(path_t * p, float step)
	{
		init( p->dimension );
		float x = 0;
		for ( int i=0; i<vert_cnt; ++i)
		{
			x += step;
			verts[i].x = x;
			verts[i].y = p->y_pos[i];
		}
	}
	void coord_2_to_1()
	{
		for( int i=0; i<vert_cnt; ++i)
			coord2_to_coord1( verts+i );
	}
} vert_path_t;

struct pso_info_t
{
	int 	gbest_id;
	path_t * pbest_path; //array
	float * y_max; //array
	float * y_min; //aray
	float * v_max_max; //array
	float * v_max_min; //array
};

class pso_plan;
struct thread_param_t
{
	pso_plan * pso_plan_ref;
	volatile int id;
};
class pso_plan
{
public:
	pso_plan();
	int pso_path_plan();
	void init_pso_conf();
	int read_pso_conf( const char * );
	int write_pso_conf( const char * );
	int read_pso_map( const char *);
	void init_pso_map();
	void calc_y_v_scale();
	void init_pso_info_path();
	bool is_crashed( const vertex_t *, const vertex_t * );
	void evolve_init_y_v(float &y, float &v, int d);
	void evolve_y_v(float &y, float &v, path_t *p, int id, int d, int k);
	int particle_evolve(path_t *p, path_t * p_tmp, int id, int k, bool do_init);
	void convert_map_backto_coord1();
	void draw_obstacle(QPainter & painter);
	void draw_dimension(QPainter & painter);
	void save_evolve_hist(const char * pic_path);
	void save_particle_pic(const char *);
	void save_fitness_pic(const char *);
public:
	friend void * particle_thread( void * );
protected:
private:
	pso_conf_t conf;
	pso_map_t map;
	pso_info_t info;
	path_t * path;
	vert_path_t * vert_path;
//	vector< vector<vert_path_t> > vert_path_all;
	vector<vert_path_t>  vert_path_hist;
	line_t * dim_lines;
	vertex_t start;
	vertex_t end;
	float step;
	vector< vector<float> > fitness_vt;
	
	int SCR_HEIGHT;
	int SCR_WIDTH;
};


#endif //PSO_PLAN_H

