#ifndef TANSA_GRAPHICS_DRAW_H_
#define TANSA_GRAPHICS_DRAW_H_
// TODO: Rename that graphics.h

#include "window.h"
#include "mesh.h"
#include "polygon.h"
#include "shader.h"


namespace tansa {
namespace graphics {

	inline void Init(int *argcp, char *argv[]){
		glutInit(argcp, argv);
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_3_2_CORE_PROFILE);
		glutInitWindowSize(512, 512);
		glutIdleFunc(window_idle);
	};

	inline void Run(){ glutMainLoop(); };

}
}



#endif
