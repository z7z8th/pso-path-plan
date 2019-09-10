#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <sys/stat.h>
#include <errno.h>
#include <assert.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <QPixmap>
#include <QPainter>
#include <QPointF>
#include "pso_plan.h"

vertex_t offset;  //offset for coordinate convert
float cosA, sinA;
int debug_threshold_level = 5;
extern int SCR_HEIGHT;
extern int SCR_WIDTH;

int level_debug(int level, char const*__restrict fmt, ...)
{
		if(level > debug_threshold_level)
				return 0;
		int ret;  
		va_list ap;  
		va_start(ap,fmt);  
		ret=vfprintf(stdout,fmt,ap);  
		va_end(ap);  
		if(level == 0)
			exit(1);
		fflush(stdout);
		return(ret);
}

pso_plan::pso_plan()
{
}

void pso_plan::init_pso_conf()
{
		conf.inertia_weight_max = 0.9f;
		conf.inertia_weight_min = 0.4f;
		conf.c1 = 2.0;
		conf.c2 = 2.0;
		conf.particle_amount = 40;
		conf.dimension = 10;
		conf.iteration_max = 50;
}

int pso_plan::read_pso_conf(const char * path)
{
	struct stat st;
	if( stat(path, &st) < 0 )
	{
		if( errno == ENOENT )
		{
			level_debug( 1, "pso config file not found, use and write default\n");
			write_pso_conf(path);
		}
		else
			level_debug( 0, "cannot stat pso config file: %s\n", strerror( errno ) );
	}
	else
	{
		int debug_level = -1;
		FILE * f_conf = fopen( path, "r" );
		if( !f_conf )
			level_debug( 0, "cannot open config file to read: %s\n" , path);
		char buf[128];
		float val = -1;
		while( EOF != fscanf( f_conf, "%s%f", buf, &val ))
		{
			level_debug( 5, "%s %.1f\n", buf, val );
			if( !strcmp("inertia_weight_max", buf ))
				conf.inertia_weight_max = val;
			else if( !strcmp("inertia_weight_min", buf ))
				conf.inertia_weight_min = val;
			else if( !strcmp( "c1", buf ))
				conf.c1 = val;
			else if( !strcmp( "c2", buf ))
				conf.c2 = val;
			else if( !strcmp( "particle_amount", buf ))
				conf.particle_amount = val;
			else if( !strcmp( "dimension", buf ))
			{
				conf.dimension = val;
				assert( conf.dimension >= 1 );
			}
			else if( !strcmp( "iteration_max", buf ))
			{
				conf.iteration_max = val;
				assert(conf.iteration_max >= 0);
			}
			else if( !strcmp( "debug_threshold_level", buf ) )
				debug_level = val;
			else
				level_debug( 0, "invalid item in config file\n\t%s\n", buf);
			val = -1;
		}
		if( debug_level != -1 && !dbg_lv_locked )
			debug_threshold_level = debug_level;
		level_debug( 3, "read pso config success\n" );
	}
	return 0;

}


int pso_plan::write_pso_conf( const char * path )
{
	FILE * f_conf = fopen( path, "w" );
	if( !f_conf )
		level_debug( 0, "cannot open config file(%s) for write",path );

	fprintf( f_conf, "%s\t%.1f\n", "inertia_weight_max", conf.inertia_weight_max );
	fprintf( f_conf, "%s\t%.1f\n", "inertia_weight_min", conf.inertia_weight_min );
	fprintf( f_conf, "%s\t\t%.1f\n", "c1", conf.c1 );
	fprintf( f_conf, "%s\t\t%.1f\n", "c2", conf.c2 );
	fprintf( f_conf, "%s\t%d\n", "particle_amount", conf.particle_amount );
	fprintf( f_conf, "%s\t%d\n", "dimension", conf.dimension );
	fprintf( f_conf, "%s\t%d\n", "iteration_max", conf.iteration_max );
	fprintf( f_conf, "%s\t%d\n", "debug_threshold_level", debug_threshold_level );

	fclose(f_conf);
	level_debug( 3, "write pso config success\n" );
	return 0;
}

int chk_next_ns_char( FILE * fp, char chk)  //check if next non-space char match
{
	char str[2];  // one fc14, one char is enough. but need two on fc9 for it store '\0'
	int ret = fscanf( fp, "%1s", str );
	if( ret == EOF || ret <= 0 || str[0] != chk )
		level_debug( 0, "invalid obstacle item, fail read \'%c\'\n", chk );
	return 0;
}
int read_one_obst( obstacle_t * obst, const int vert_cnt, FILE * fp)
{
	assert(fp);
	obst->vert_cnt = vert_cnt;
	obst->verts = new vertex_t[vert_cnt];
	if( !obst->verts )
		level_debug( 0, "fail to allocate mem for obst->verts\n" ); 
	level_debug( 5, "\t\t" );
	int ret1, ret2;
	chk_next_ns_char( fp, '{' );
	int i;
	for( i=0; i<vert_cnt; ++i)
	{
		chk_next_ns_char( fp, '(' );
		ret1 = fscanf( fp, "%f", &(obst->verts[i].x));
		chk_next_ns_char( fp, ',' );
		ret2 = fscanf( fp, "%f", &(obst->verts[i].y));
		chk_next_ns_char( fp, ')' );
	 	if( ret1 != 1 || ret2 != 1 )
			level_debug( 0, "invalid obstacle item,"
					" fail read x,y ret:%d ret2:%d\n", ret1, ret2 );
		else
			level_debug( 5, " (%.2f, %.2f)", obst->verts[i].x, obst->verts[i].y );
	}
	chk_next_ns_char( fp, '}' );
	level_debug( 5, "\n");
	return 0;
}

int pso_plan::read_pso_map( const char * path)
{
	struct stat st;
	if( stat(path, &st) < 0 )
		level_debug( 0, "cannot stat map file: %s\n", strerror( errno ) );
	FILE * f_map = fopen( path, "r" );
	if( !f_map )
		level_debug( 0, "fail to open map file: %s\n", strerror( errno ) );
	char str[2];
	char buf[128];
	int cnt = 0;
	int room_readed = 0;
	int cnt_readed = 0;
	int start_end_readed = 0;
	int obst_ind = 0;
	while( fscanf( f_map, "%1s", str ) != EOF )
	{
		if( str[0] == '#' )
		{
			(void) fscanf( f_map, "%*[^\n]" );
		}
		else
		{
			if( fseek( f_map, -1, SEEK_CUR ) == -1 )
				level_debug(0, "fail to seek back(in FUNC: %s)", __FUNCTION__);
			else if( fscanf( f_map, "%127s %d", buf, &cnt) == 2 )
			{
				level_debug( 5, "NAME CNT: %s\t%d\n", buf, cnt );
				if( cnt <= 0 )
				{
					level_debug( 0, "invalid cnt of %s:%d\n", buf, cnt );
				}
				else if( !strcmp( "obstacle", buf ) )
				{
					if( room_readed )
					{
						if( map.obst_cnt == obst_ind && obst_ind != 0 )
						{
							level_debug( 0, "too much obstacles\n" );
						}
						else if( cnt_readed )
						{
							read_one_obst( map.obsts + obst_ind, cnt, f_map );
							++obst_ind;
						}
						else
							level_debug( 0, "obstacle_cnt must come first\n");
					}
					else
						level_debug( 0, "room must come before obstacles\n" );
				}
				else if ( !strcmp( "room", buf ) )
				{
					if( room_readed )
						level_debug( 0, "too much room item\n" );
					else if( cnt_readed )
					{
						read_one_obst( map.obsts + obst_ind, cnt, f_map );
						++obst_ind;
						room_readed = 1;
					}
					else
						level_debug( 0, "obstacle_cnt must come first\n");
				}
				else if( !strcmp( "obstacle_cnt", buf ))
				{

					map.obst_cnt = cnt + 1;
					map.obsts = new obstacle_t[ cnt + 1 ];
					if( !map.obsts )
						level_debug( 0, "fail to allocate mem for obstacles\n" );
					cnt_readed = 1;
				}
				else if( !strcmp( "start_end", buf))
				{
					obstacle_t se_obst;
					read_one_obst( &se_obst, 2, f_map );
					start = se_obst.verts[0];
					end = se_obst.verts[1];
					delete []se_obst.verts;
					start_end_readed = 1;
				}
			}
			else
				level_debug( 0, "invalid obstacle item, fail read ..NAME CNT\n" );

		}
	}
	if( !start_end_readed )
		level_debug(0, "start end not found in map!\n");
	return 0;
}

void coord1_to_coord2( vertex_t * vert )
{
	vertex_t tmp;
	tmp.x = cosA * (vert->x - offset.x) + sinA * (vert->y - offset.y);
	tmp.y = cosA * (vert->y - offset.y) - sinA * (vert->x - offset.x);
	*vert = tmp;
	level_debug( 7, "val after convert to coord2: %.2f, %.2f\n", vert->x, vert->y );
}

void coord2_to_coord1( vertex_t * vert )
{
	vertex_t tmp;
	tmp.x = cosA * (vert->x - offset.x) - sinA * (vert->y - offset.y);
	tmp.y = cosA * (vert->y - offset.y) + sinA * (vert->x - offset.x);
	*vert = tmp;
	level_debug( 7, "val after convert to coord1: %.2f, %.2f\n", vert->x, vert->y );
}

void calc_coef( coefficient_t * coef, vertex_t * vert1, vertex_t * vert2 )
{
	if(  vert1->x == vert2->x )
	{
		coef->a = 1;
		coef->b = 0;
		coef->c = -vert1->x;
	}
	else
	{
		coef->a = - (vert2->y - vert1->y) / (vert2->x - vert1->x);
		coef->b = 1;
		coef->c = - coef->a * vert1->x - vert1->y;
	}
	level_debug( 5, "coefs: %.4f, %.4f, %.4f\n", coef->a, coef->b, coef->c );
}

void pso_plan::init_pso_map()
{
	offset = start;
	float dist_se = sqrtf( sq( start.x - end.x ) + sq( start.y - end.y ) );
	cosA = ( end.x - start.x ) / dist_se;
	sinA = ( end.y - start.y ) / dist_se;
	level_debug( 4, "cosA: %.4f , sinA: %.4f\n", cosA, sinA );
	for( int i=0; i<map.obst_cnt; ++i)
	{
		for( int j=0; j<map.obsts[i].vert_cnt; ++j)
			coord1_to_coord2( map.obsts[i].verts+j );
		map.obsts[i].print();
		map.obsts[i].coefs = new coefficient_t[ map.obsts[i].vert_cnt];
		assert( map.obsts[i].coefs );
	}
	level_debug( 5, "convert start end:\n" );
	coord1_to_coord2( &start );
	coord1_to_coord2( &end );
	vertex_t vert_tmp;
	vert_tmp.x = vert_tmp.y = 0;
	coord1_to_coord2( &vert_tmp );
	offset = vert_tmp;
	step = (end.x - start.x)/(conf.dimension + 1);
	level_debug( 5, "start: %.2f, %.2f\n", start.x, start.y );
	level_debug( 5, "end: %.2f, %.2f\n", end.x, end.y );
	level_debug( 5, "step: %.2f\n", step);
	level_debug( 5, "offset: %.2f, %.2f\n", offset.x, offset.y);

	for( int i=0; i<map.obst_cnt; ++i)
	{
		level_debug( 5, "------- coefs, obstacle %d -------\n", i);
		for( int j=0; j<map.obsts[i].vert_cnt; ++j)
		{
			calc_coef( map.obsts[i].coefs+j, 
					map.obsts[i].verts + j,
				   	map.obsts[i].verts + (j + 1) % map.obsts[i].vert_cnt );
		}
	}
}

inline float& max(float f1, float f2)
{
	return (f1 > f2 ? f1 : f2);
}
inline float& min(float f1, float f2)
{
	return (f1 < f2 ? f1 : f2);
}

bool in_range(float rang1, float rang2, float val)
{
	if( (val <= rang2 + delta && val >= rang1 - delta ) 
			|| (val <= rang1 + delta && val >= rang2 - delta) )
		return true;
	else 
		return false;
}

#define VELOCITY_RATIO_MAX 0.10f
#define VELOCITY_RATIO_MIN 0.01f

void pso_plan::calc_y_v_scale()
{
	info.y_max = new float[ conf.dimension ];
	info.y_min = new float[ conf.dimension ];
	info.v_max_max = new float[ conf.dimension ];
	info.v_max_min = new float[ conf.dimension ];
	assert(info.y_max && info.y_min && info.v_max_max && info.v_max_min);
	memset( info.y_max, 0, conf.dimension * sizeof(float));  //0就行了，路径一定在房间内
	memset( info.y_min, 0, conf.dimension * sizeof(float));
	memset( info.v_max_max, 0, conf.dimension * sizeof(float));
	memset( info.v_max_min, 0, conf.dimension * sizeof(float));
	float px = 0;
	float py = 0;
	obstacle_t & room = map.obsts[0];
	for( int i=0; i<conf.dimension; ++i)
	{
		px += step;
		for( int j=0; j<room.vert_cnt; ++j)
		{
			if( px <= max( room.verts[j].x, room.verts[(j+1)%room.vert_cnt].x) 
				&& px >= min( room.verts[j].x, room.verts[(j+1)%room.vert_cnt].x) )
			{
				//level_debug(3, "warning: can\'t get y_max_min from room line %d \n", j);
				py = - room.coefs[j].a * px - room.coefs[j].c;
				if( py > info.y_max[i])
					info.y_max[i] = py;
				else if( py < info.y_min[i])
					info.y_min[i] = py;
			}
		}
		info.v_max_max[i] = fabs(VELOCITY_RATIO_MAX * (info.y_max[i] - info.y_min[i]));
		info.v_max_min[i] = fabs(VELOCITY_RATIO_MIN * (info.y_max[i] - info.y_min[i]));
		level_debug( 3, "y_max[%d] = %.2f, y_min[%d] = %.2f, v_max_max[%d] = %.2f, v_max_min[%d] = %.2f\n",
			   	i, info.y_max[i],
			   	i, info.y_min[i],
				i, info.v_max_max[i],
				i, info.v_max_min[i]);
	}
}

void pso_plan::init_pso_info_path()
{
	path = new path_t [conf.particle_amount];
	info.pbest_path = new path_t [conf.particle_amount];
	assert( path && info.pbest_path );
	for( int i=0; i<conf.particle_amount; ++i)
	{
		path[i].init(conf.dimension);
		info.pbest_path[i].init(conf.dimension, false);
	}
	info.gbest_id = -1;
}

bool pso_plan::is_crashed( const vertex_t *ps, const vertex_t * pe)
{
	float a = - (pe->y - ps->y) / (pe->x - ps->x);
	float b = 1;
	float c = - a * ps->x - ps->y;
	float x, y;
	x = y = 0;
	obstacle_t * obst;

	for( int i=0; i<map.obst_cnt; ++i)
	{
		obst = map.obsts+i;
		for( int j=0; j<obst->vert_cnt; ++j)
		{
			if( !(a == obst->coefs[j].a && b == obst->coefs[j].b))
			{
				if( obst->coefs[j].b == 0 )
					x = - obst->coefs[j].c / obst->coefs[j].a;
				else
					x = ( c - obst->coefs[j].c )/(obst->coefs[j].a - a);
				y = -c - a*x;
				if( (
					x <= max(obst->verts[j].x, obst->verts[(j+1) % obst->vert_cnt].x) + delta
					&& x >= min(obst->verts[j].x, obst->verts[(j+1) % obst->vert_cnt].x) - delta
					&& y <= max(obst->verts[j].y, obst->verts[(j+1) % obst->vert_cnt].y) + delta
					&& y >= min(obst->verts[j].y, obst->verts[(j+1) % obst->vert_cnt].y) - delta
					)
				 	&& 
					( 
					 	x <= max(ps->x, pe->x) + delta
						&& x >= min(ps->x, pe->x) - delta
						&& y <= max(ps->y, pe->y) + delta
						&& y >= min(ps->y, pe->y) - delta
					)
				  )
				{
					level_debug( 6, "%s sorry: %d, %d (%.2f, %.2f)\n", __FUNCTION__, i, j, x, y);
					return true;
				}
			}
		}
	}
	level_debug( 6, "%s: ok (%.2f, %.2f)\n", __FUNCTION__, x, y);
	return false;
}


pthread_mutex_t info_mutex;
pthread_mutex_t finish_cnt_mutex;
pthread_mutex_t rand_mutex;

void pso_plan::evolve_init_y_v(float &y, float &v, int d)
{
//	level_debug( 5, "%s\n", __FUNCTION__);
	pthread_mutex_lock(&rand_mutex);
	float ry = (float) rand()/RAND_MAX;
	float rv = (float) rand()/RAND_MAX;
	pthread_mutex_unlock(&rand_mutex);
	level_debug( 7, "ry: %.2f, rv: %.2f\n", ry, rv);
	y = info.y_min[d] + ry * (info.y_max[d] - info.y_min[d]);
	v = (rv * 2 - 1) * info.v_max_max[d];
	level_debug(5, "y: %.2f, v: %.2f\n", y, v);
}

void pso_plan::evolve_y_v(float &y, float &v, path_t *p, int id, int d, int k)
{
	float w = conf.inertia_weight_max - 
		(float) k * (conf.inertia_weight_max - conf.inertia_weight_min)/conf.iteration_max;
	float r1 = 0;
	float r2 = 0;
	float r_w_ga = 0;
	float Pm_ga = 0.2;
	pthread_mutex_lock(&rand_mutex);
	r1 = (float) rand()/RAND_MAX;
	r2 = (float) rand()/RAND_MAX;
	r_w_ga = (float) rand()/RAND_MAX;
	pthread_mutex_unlock(&rand_mutex);
	w = w*(1 - Pm_ga * (r_w_ga - 1) );

	pthread_mutex_lock(&info_mutex);
	v = w * p->velocity[d] 
		+ conf.c1 * r1 * (info.pbest_path[id].y_pos[d] - p->y_pos[d])
		+ conf.c2 * r2 * (info.pbest_path[info.gbest_id].y_pos[d] - p->y_pos[d]);
	level_debug( 5, "- id %d, v[k-1] %.2f, pbest_y %.2f, gbest_y %.2f, p_y %.2f, __v %.2f\n", 
			id,
			p->velocity[d], 
			info.pbest_path[id].y_pos[d],
		   	info.pbest_path[info.gbest_id].y_pos[d],
		   	p->y_pos[d], 
			v);
	pthread_mutex_unlock(&info_mutex);

	//float v_max = info.v_max_max[d]
	//  			 	- (float)k * (info.v_max_max[d] - info.v_max_min[d]) / conf.iteration_max;
	// v_max成反比减小
	float v_max = (info.v_max_max[d] - info.v_max_min[d])/(k+1) + info.v_max_min[d];
	if( v > v_max )
		v = v_max - delta;
	else if( v < -v_max)
		v = - v_max + delta;

	y = p->y_pos[d] + v;
	if( y > info.y_max[d] )
		y = info.y_max[d] -delta;
	else if( y < info.y_min[d] )
		y = info.y_min[d] + delta;
	level_debug(5, "id %d, w %.2f, r1 %.2f, r2 %.2f, y %.2f, v %.2f, "
			"y_max %.2f, y_min %.2f, v_max %.2f\n",
		   	id, w, r1, r2, y, v, info.y_max[d], info.y_min[d], v_max);
}

int finish_cnt;
#define EVOLVE_RETRY 40
#define FALLBACK_RETRY 20

int pso_plan::particle_evolve(path_t *p, path_t *p_tmp, int id, int k, bool do_init)
{
	assert(p);
	assert(( !p_tmp && k==-1 && do_init)||(p_tmp && k>=0 && !do_init));
	vertex_t ps, pe;
	float y, v;
	int ret;
	int fallback_cnt=0;
	if( do_init )
		p_tmp = p;

	for( int d=0; d < p->dimension; ++d)
	{
		for( int i=0; i<EVOLVE_RETRY; ++i)
		{
			if( do_init )
				evolve_init_y_v( y, v, d );
			else
				evolve_y_v( y, v, p, id, d, k);
			if( 0 == d )
			{
				ps = start;
				pe.x = step;
				pe.y = y;
			}
			else
			{
				ps.x = step * d;
				ps.y = p_tmp->y_pos[d-1];
				pe.x = ps.x + step;
				pe.y = y;
			}
			ret = is_crashed(&ps, &pe);
			if(!ret &&  d == p->dimension - 1)
			{
				ps = pe;
				pe = end;
				ret = is_crashed(&ps, &pe);
				if(!ret){	break;	}
			}
			else if( !ret )
			{	break;	}

			if(ret)
			{
				level_debug( 6, "^^^^^^^^%s id %d, iterate %d, fallback %d, retry %d ^^^^^^^^\n",
					   	do_init ? "init":"evolve", id, k, fallback_cnt, i+1);
			}
		}

		if(ret)
		{
			level_debug( 3, "---- fail to %s particle: %d, dimension: %d,"
					" will fallback %d ---\n",
				   	do_init ? "init":"evolve", id, d, fallback_cnt+1 );
			if( fallback_cnt++ >= FALLBACK_RETRY )
			{
				level_debug( 2, "fail to %s particle[%d] dimension[%d] after "
					"fallback %d times, will die\n", 
					do_init ? "init":"evolve", id, d, fallback_cnt-1 );
				return 1;
			}
			else{	d = -1; }
		}
		else
		{
			p_tmp->y_pos[d] = y;
			p_tmp->velocity[d] = v;
			//fallback_cnt = 0;
			level_debug( 5, "id %d, iterate %d, dimension[%d], good pos (%.2f, %.2f)\n",
				   	id, k, d, pe.x, pe.y);
		}
	}
	if( !do_init )
	{
		memcpy( p->y_pos, p_tmp->y_pos, p->dimension * sizeof(p->y_pos[0]));
		memcpy( p->velocity, p_tmp->velocity, p->dimension * sizeof(p->velocity[0]));
	}
	return 0;
}

inline void update_fitness(path_t *p, path_t *pb, int id, pso_info_t * pi, bool do_init)
{
	pthread_mutex_lock(&info_mutex);
	if( p->fitness < pb->fitness || do_init)
	{
		pb->clone(p);  //this should in the lock, for other thread maybe reading this
	}

	if( pi->gbest_id == -1 || p->fitness < pi->pbest_path[pi->gbest_id].fitness)
		pi->gbest_id = id;
	pthread_mutex_unlock(&info_mutex);
	level_debug( 4, "update gbest_id to %d\n", id);
}

#define DBG_ID 0
#define FITNESS_CHNG_MIN 1
#define FITNESS_UNCHNG_CNT_MAX 10
void * particle_thread( void * tparam ) //remember to delete tparam
{
	thread_param_t *param = (thread_param_t *) tparam;
	pso_plan * pso_plan_ref = (pso_plan*)param->pso_plan_ref;
	pso_conf_t * conf = &pso_plan_ref->conf;
	pso_info_t * info = &pso_plan_ref->info;
	int id = param->id;
	level_debug( 5, "thread id: %d\n", id);
	path_t * this_path = pso_plan_ref->path+id;
	path_t * this_path_best = info->pbest_path+id;
	vector<float> & this_fitness_vt = pso_plan_ref->fitness_vt[id];
	vector<vert_path_t> * this_vert_path_hist;
	if( id == DBG_ID )
		this_vert_path_hist = & pso_plan_ref->vert_path_hist;
	float step = pso_plan_ref->step;
	int iteration_max = conf->iteration_max;

	if( 0 == pso_plan_ref->particle_evolve(this_path, NULL, id, -1, true) )
	{
		level_debug(4, "init particle success\n");
		this_path->calc_fitness(step);
		level_debug( 5, "this_path->fitness: %.2f\n", this_path->fitness);
		update_fitness( this_path, this_path_best, id, info, true);
		this_fitness_vt.push_back(this_path_best->fitness);
		if( id == DBG_ID  )
		{
			vert_path_t vert_path;
			vert_path.clone(this_path, step);
			this_vert_path_hist->push_back(vert_path);
		}

		///*
		path_t * this_path_tmp = new path_t;
		this_path_tmp->init(this_path->dimension, true);
		int fitness_unchng_cnt = 0;
		float last_fitness = this_path->fitness;
		for( int k=0; k<iteration_max; ++k)
		{
			level_debug( 4, "\n----------------- k %d ---------------------\n\n", k);
			if(pso_plan_ref->particle_evolve(this_path, this_path_tmp, id, k, false))
			{
				this_path->isbad = true;
				//this_path_best->isbad = true;
				level_debug( 2, "************ particle %d dead ************\n", id);
				break;
			}
			this_path->calc_fitness(step);
			update_fitness( this_path, this_path_best, id, info, false);
			this_fitness_vt.push_back(this_path_best->fitness);
			if( id == DBG_ID  )
			{
				vert_path_t vert_path;
				vert_path.clone(this_path, step);
				this_vert_path_hist->push_back(vert_path);
			}
			if( fabs(last_fitness - this_path->fitness) < FITNESS_CHNG_MIN )
				++ fitness_unchng_cnt;
			if( fitness_unchng_cnt > FITNESS_UNCHNG_CNT_MAX )
				break;
			last_fitness = this_path->fitness;
		}
		this_path_tmp->destroy();
		delete this_path_tmp;
		//*/
	}
	else
	{
		this_path_best->isbad = true;
		level_debug( 3, "*********** fail to init particle %d\n, will die", id);
	}
	pthread_mutex_lock( &finish_cnt_mutex );
	++finish_cnt;
	pthread_mutex_unlock( &finish_cnt_mutex );
	delete (thread_param_t *)tparam;
	return NULL;
}

int pso_plan::pso_path_plan()
{
	fitness_vt.resize(conf.particle_amount);
	pthread_mutex_init(&info_mutex, NULL);
	pthread_mutex_init(&finish_cnt_mutex, NULL);
	pthread_mutex_init(&rand_mutex, NULL);

	srand(time(NULL));
	//rand() is not thread safe, so lock it, this is a test
	pthread_mutex_lock(&rand_mutex);
	float tmp_r = rand();
	level_debug(5, "rand: %f\n", tmp_r);
	pthread_mutex_unlock(&rand_mutex);
	assert( tmp_r <= RAND_MAX && tmp_r >= 0);

	finish_cnt = 0;
	for( int i=0; i<conf.particle_amount; ++i )
	{
		thread_param_t * tparam = new thread_param_t;
		tparam->pso_plan_ref = this;
		tparam->id = i;
		pthread_t thread;
		if (pthread_create( &thread, NULL, particle_thread, tparam))
		{
			level_debug( 0, "fail to create thread: %d\n", i);
		}
	}
	while( finish_cnt < conf.particle_amount )
		usleep(1000);
	pthread_mutex_destroy(&info_mutex);
	pthread_mutex_destroy(&finish_cnt_mutex);
	pthread_mutex_destroy(&rand_mutex);
	level_debug(1, "best fitness: info.pbest_path[%d].fitness = %.2f\n",
		   	info.gbest_id, info.gbest_id == -1 ? -1 : info.pbest_path[info.gbest_id].fitness);

	return 0;
}

void pso_plan::convert_map_backto_coord1()
{
	level_debug( 6, "offset: (%.2f, %.2f)\n", offset.x, offset.y);
	obstacle_t * obst;
	for( int i=0; i<map.obst_cnt; ++i)
	{
		obst = map.obsts+i;
		level_debug( 6, "after convert back, ");
		for( int j=0; j<obst->vert_cnt; ++j)
		{
			coord2_to_coord1( obst->verts+j );
		}
		obst->print();
	}
	coord2_to_coord1( &start );
	coord2_to_coord1( &end );
	vert_path = new vert_path_t[conf.particle_amount];
	assert( vert_path );
	for( int i=0; i<conf.particle_amount; ++i)
	{
		vert_path[i].clone( info.pbest_path+i, step );
		vert_path[i].coord_2_to_1();
	}
	dim_lines = new line_t[conf.dimension];
	assert(dim_lines);
	float dim_ln_x = 0;
	for( int d=0; d<conf.dimension; ++d)
	{
		dim_ln_x += step;
		dim_lines[d].vert_s.x = dim_ln_x;
		dim_lines[d].vert_s.y = info.y_min[d];
		dim_lines[d].vert_e.x = dim_ln_x;
		dim_lines[d].vert_e.y = info.y_max[d];
		coord2_to_coord1( &dim_lines[d].vert_s );
		coord2_to_coord1( &dim_lines[d].vert_e );
		level_debug( 6, "1 dim lines[%d]: (%.2f, %.2f) (%.2f, %.2f)\n", d, 
				dim_lines[d].vert_s.x, dim_lines[d].vert_s.y,
				dim_lines[d].vert_e.x, dim_lines[d].vert_e.y);
	}	
	for( unsigned int i=0; i < vert_path_hist.size(); ++i)
	{
		vert_path_hist.at(i).coord_2_to_1();
	}

}


float zoom;

void pso_plan::draw_obstacle(QPainter & painter)
{
	QPolygonF obst_polygon;
	QPointF obst_point;
	QPainterPath painter_path;
	obstacle_t * obst;
	for( int i=1; i<map.obst_cnt; ++i)
	{
		obst = map.obsts+i;
		obst_polygon.resize(obst->vert_cnt);
		for( int j=0; j<obst->vert_cnt; ++j)
		{
			obst_polygon.replace( j, QPointF(obst->verts[j].x * zoom,
					   	obst->verts[j].y * zoom )); 
		}
		painter_path.addPolygon( obst_polygon );
		painter.fillPath(painter_path, QBrush(Qt::gray));
	}
}

void pso_plan::draw_dimension(QPainter & painter)
{
	painter.setPen(Qt::yellow);
	for( int d=0; d<conf.dimension; ++d)
	{
		painter.drawLine(dim_lines[d].vert_s.x*zoom,
			   	dim_lines[d].vert_s.y*zoom,
			   	dim_lines[d].vert_e.x*zoom,
			   	dim_lines[d].vert_e.y*zoom);
	}	
	painter.drawLine(start.x * zoom, start.y * zoom,
		   	end.x * zoom, end.y * zoom);
}
void pso_plan::draw_evolve_hist(const char * pic_path)
{
	obstacle_t * room = &map.obsts[0];
	float width = max(fabs(room->verts[1].x - room->verts[0].x),
						fabs(room->verts[2].x - room->verts[1].x));
	float height = max(fabs(room->verts[1].y - room->verts[0].y),
						fabs(room->verts[2].y - room->verts[1].y));
	zoom = (float)SCR_HEIGHT / height;

	const Qt::GlobalColor color_arr[] = 
	{
		Qt::white,
		Qt::red,
		Qt::darkRed,
//		Qt::green,
		Qt::darkGreen,
		Qt::blue,
		Qt::darkBlue,
		Qt::cyan,
		Qt::darkCyan,
		Qt::magenta,
		Qt::darkMagenta,
		Qt::yellow,
		Qt::darkYellow,
		Qt::gray,
		Qt::darkGray,
		Qt::lightGray
	};
	const int color_cnt = sizeof(color_arr)/sizeof(color_arr[0]);
    QPixmap evolve_pic(width*zoom , height*zoom);
    QPainter painter(&evolve_pic);
	painter.fillRect(0, 0, SCR_WIDTH, SCR_HEIGHT, Qt::black);
	draw_obstacle(painter);
	QPointF * points = new QPointF[conf.dimension+2];
	points[0].setX(start.x * zoom );
	points[0].setY(start.y * zoom );
	points[conf.dimension+1].setX(end.x * zoom );
	points[conf.dimension+1].setY(end.y * zoom );
	level_debug( 4, "evolve_vert_path size: %d\n", vert_path_hist.size());
	for( unsigned int i=0; i < vert_path_hist.size(); ++i)
	{
		level_debug( 5, "vert_path[%d].size: %d\n", i, vert_path_hist.at(i).vert_cnt);
		for( int j=0; j<conf.dimension; ++j)
		{
			points[j+1].setX(vert_path_hist.at(i).verts[j].x*zoom);
			points[j+1].setY(vert_path_hist.at(i).verts[j].y*zoom);
		}
		painter.setPen(color_arr[i%color_cnt]);
//		painter.setPen(Qt::white);
		painter.drawPolyline( points, conf.dimension + 2 );
	}
	painter.setPen(Qt::green);
	for( int j=0; info.gbest_id != -1 && j<vert_path[info.gbest_id].vert_cnt; ++j)
	{
		points[j+1].setX( vert_path[info.gbest_id].verts[j].x * zoom );
		points[j+1].setY( vert_path[info.gbest_id].verts[j].y * zoom );
	}
	painter.drawPolyline( points, conf.dimension + 2);
	delete []points;
	draw_dimension(painter);
	evolve_pic.save(pic_path);
}

void pso_plan::draw_particle_pic(const char * pic_path)
{

	obstacle_t * room = &map.obsts[0];
	float width = max(fabs(room->verts[1].x - room->verts[0].x),
						fabs(room->verts[2].x - room->verts[1].x));
	float height = max(fabs(room->verts[1].y - room->verts[0].y),
						fabs(room->verts[2].y - room->verts[1].y));
	zoom = (float)SCR_HEIGHT / height;

    QPixmap result_pic(width*zoom , height*zoom);
    QPainter painter(&result_pic);
	painter.fillRect(0, 0, SCR_WIDTH, SCR_HEIGHT, Qt::black);

	painter.setPen(Qt::white);
	QPointF * points;

	draw_obstacle(painter);

	points = new QPointF[conf.dimension+2];
	assert(points);
	points[0].setX(start.x * zoom );
	points[0].setY(start.y * zoom );
	points[conf.dimension+1].setX(end.x * zoom );
	points[conf.dimension+1].setY(end.y * zoom );
	painter.setPen(Qt::blue);
	for( int i=0; i<conf.particle_amount; ++i)
	{
		if( i == info.gbest_id )
			continue;
		if( info.pbest_path[i].isbad )
			continue;
		for( int j=0; j<vert_path[i].vert_cnt; ++j)
		{
			points[j+1].setX( vert_path[i].verts[j].x * zoom );
			points[j+1].setY( vert_path[i].verts[j].y * zoom );
		}
		painter.drawPolyline( points, conf.dimension + 2);
	}
	painter.setPen(Qt::green);
	for( int j=0; info.gbest_id != -1 && j<vert_path[info.gbest_id].vert_cnt; ++j)
	{
		points[j+1].setX( vert_path[info.gbest_id].verts[j].x * zoom );
		points[j+1].setY( vert_path[info.gbest_id].verts[j].y * zoom );
	}
	painter.drawPolyline( points, conf.dimension + 2);
	delete []points;

	draw_dimension(painter);

	if( info.gbest_id == -1 )
	{
		painter.setPen(Qt::red);
		painter.setFont(QFont("", 60));
		painter.drawText(0.35 * width * zoom, 0.48 * height * zoom, "FAIL!!!");
	}
    result_pic.save(pic_path);
}

void pso_plan::draw_fitness_pic(const char *pic_path)
{
	QPixmap fitness_pic(SCR_WIDTH, SCR_HEIGHT);
	QPainter painter( &fitness_pic );
	painter.fillRect(0, 0, SCR_WIDTH, SCR_HEIGHT, Qt::black);
	const float zoom = SCR_HEIGHT/(3.0f * info.pbest_path[info.gbest_id].fitness);
	level_debug( 5, "zoom: %f\n", zoom );
	float scale_bar_width = 35;
	float scale_div = 10;
	float scale_val_y = 0;
	float scale_step = SCR_HEIGHT / zoom / scale_div;
	QString str_val_y;
	painter.setPen(Qt::white);
	for(int i=0; i<scale_div; ++i)
	{
		painter.drawText( 0, SCR_HEIGHT - scale_val_y*zoom, str_val_y.setNum((int)scale_val_y) );
		scale_val_y += scale_step;
	}

	float fitness_x;
	float fitness_x_step = (float) (SCR_WIDTH - scale_bar_width) / (conf.iteration_max);
	painter.setPen(Qt::yellow);
	fitness_x = scale_bar_width;
	for( int i=0; i<conf.iteration_max; ++i )
	{
		painter.drawLine(fitness_x, 0, fitness_x, SCR_HEIGHT);
		fitness_x += fitness_x_step;
	}
	painter.setPen(Qt::white);
	for( int i=0; i<conf.particle_amount; ++i )
	{
		level_debug(4, "fitness_vt[%d].size(): %d\n", i, fitness_vt[i].size());
		fitness_x = scale_bar_width;
		for( unsigned int j=0; j+1 < fitness_vt[i].size(); ++j )
		{
			painter.drawLine(fitness_x, SCR_HEIGHT - fitness_vt[i].at(j) * zoom,
				   	fitness_x + fitness_x_step, SCR_HEIGHT - fitness_vt[i].at(j+1) * zoom);
			fitness_x += fitness_x_step;
		}
	}
	fitness_pic.save(pic_path);
}

void util_draw_plain_coord()
{
	QPixmap coord(1200, 800);
	QPainter painter2(&coord);
	painter2.fillRect(0,0,1200,800, Qt::black);
	painter2.setPen(Qt::yellow);
	QString num;
	for( int i=1; i<16; ++i)
	{
		painter2.drawLine(0, 50*i, 1200, 50*i);
		if( i%2 == 0 )
		{
			painter2.drawText(2, i*50+13,  num.number(i*50));
		}
	}
	for( int i=1; i<24; ++i)
	{
		painter2.drawLine(i*50, 0, i*50, 800);
		if( i%2 == 0 )
		{
			painter2.drawText(i*50+2, 10, num.number(i*50));
		}
	}
	coord.save("coord.png");
}
