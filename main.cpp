#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <QApplication>
#include <QDesktopWidget>
#include "pso_plan.h"

bool dbg_lv_locked = false;
int SCR_HEIGHT;
int SCR_WIDTH;

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
	int scr_border_size = 90;
	SCR_HEIGHT = QApplication::desktop()->height() - scr_border_size;
	SCR_WIDTH = QApplication::desktop()->width();
	char * argv0 = (char *) malloc( strlen(argv[0])+1 );
	strcpy(argv0, argv[0]);

	char * path = basename(argv0);
	char config_file_path[FILENAME_MAX] = "pso.conf";
	char map_file_path[FILENAME_MAX] = "room.map";
	bool  have_config_file = 0;
	bool  have_map_file = 0;
	int ch;
	opterr = 0;
	while( (ch = getopt( argc, argv, "c:m:d:h")) != -1 )
	{
		 //printf("optind: %d\n", optind);
		 switch( ch )
		 {
			 case 'c':
				 if( strlen( optarg ) >= FILENAME_MAX )
				 {
					 level_debug( 0, "config file pathlen >= FILENAME_MAX(%d)\n", FILENAME_MAX );
				 }
				 have_config_file = 1;
				 strcpy( config_file_path, optarg );
				 break;
			case 'm':
				 if( strlen( optarg ) >= FILENAME_MAX )
				 {
					 level_debug( 0, "map file pathlen >= FILENAME_MAX(%d)\n", FILENAME_MAX );
				 }
				 have_map_file = 1;
				 strcpy( map_file_path, optarg );
				 break;
   			case 'd':
				 {
				 int debug_level = -1;
				 if( sscanf(optarg, "%d", &debug_level ) != 1 )
					 level_debug( 0, "cannot get debug level from %s\n", optarg );
				 if( debug_level < 0 )
					 level_debug( 0, "invalid debug level: %d\n", debug_level );
				 if( !dbg_lv_locked )
				 {
					 debug_threshold_level = debug_level;
					 dbg_lv_locked = true;
				 }
				 break;
				 }
			case 'h':
			default:
		 		 level_debug(0, "usage: %s [-h] [-c config_file] [-m map_file] [-d debug_level]\n", path);
		 };
	}
	if( !have_config_file )
	{
		level_debug(4, "no config file specialed, use default: pso.conf\n" );
	}
	if( !have_map_file)
	{
		level_debug(4, "no map file specialed, use default: room.map\n" );
	}
	level_debug( 5, "config: %s\nmap: %s\n", config_file_path, map_file_path );
	pso_plan pso_planer;
	pso_planer.init_pso_conf();
	pso_planer.read_pso_conf( config_file_path );
	pso_planer.read_pso_map( map_file_path );
	pso_planer.init_pso_map();
	pso_planer.calc_y_v_scale();
	pso_planer.init_pso_info_path();
	pso_planer.pso_path_plan();
	pso_planer.convert_map_backto_coord1();
	const char *result_pic_path = "particle.png";
	const char *fitness_pic_path = "fitness.png";
	const char *evolve_pic_path = "evolve.png";
	pso_planer.draw_particle_pic(result_pic_path);
	pso_planer.draw_fitness_pic(fitness_pic_path);
	pso_planer.draw_evolve_hist( evolve_pic_path );

	return 0;
}


