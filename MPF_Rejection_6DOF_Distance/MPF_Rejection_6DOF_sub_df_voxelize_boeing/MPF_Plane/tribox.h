#ifndef TRIBOX_H
#define TRIBOX_H

/* Triangle-box intersection algorithm */
/* triBoxOverlap(double boxcenter[3], double boxhalfsize[3], double triverts[3][3]) */
/* return 1 if overlap, else return 0*/

int planeBoxOverlap(double normal[3], double vert[3], double maxbox[3]);
int triBoxOverlap(double boxcenter[3], double boxhalfsize[3], double triverts[3][3]);

#endif // TRIBOX_H