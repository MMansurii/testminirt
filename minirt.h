#ifndef MINIRT_H
# define MINIRT_H

# include "../MLX42/include/MLX42/MLX42.h"
# include "../libft/libft.h"
# include <fcntl.h>
# include <math.h>
# include <stdio.h>
# include <stdlib.h>

/* ----------------------------*/ /*types*/

enum					e_vec3_op
{
	SUB,
	ADD,
	MUL,
	CROSS,
};

enum					e_object_type
{
	SPHERE,
	PLANE,
	CYLINDER
};

typedef struct s_quadratic
{
	float				a;
	float				b;
	float				c;
	float				delta;
	float				t1;
	float				t2;
}						t_quadratic;

typedef struct s_vec3
{
	float				x;
	float				y;
	float				z;
}						t_vec3;

typedef struct s_color
{
	int					r;
	int					g;
	int					b;
}						t_color;

/* ----------------------------*/ /*objects*/

typedef struct s_sphere
{
	t_vec3				pos;
	float				radius;
	t_color				color;
}						t_sphere;

typedef struct s_plane
{
	t_vec3				pos;
	t_vec3				normal;
	t_color				color;
}						t_plane;

typedef struct s_cylinder
{
	t_vec3				pos;
	t_vec3				normal;
	float				radius;
	float				height;
	t_color				color;
	t_vec3				cap1;
	t_vec3				cap2;
}						t_cylinder;

typedef struct s_object
{
	t_sphere			sphere;
	t_plane				plane;
	t_cylinder			cylinder;
	enum e_object_type	type;
	int					id;
}						t_object;

/* ----------------------------*/ /*scene*/

typedef struct s_viewport
{
	float				w;
	float				h;
}						t_viewport;

typedef struct s_canvas
{
	float				w;
	float				h;
}						t_canvas;

typedef struct s_camera
{
	t_vec3				pos;
	t_vec3				normal;
	float				fov;
}						t_camera;

typedef struct s_ambient
{
	float				ratio;
	t_color				color;
}						t_ambient;

typedef struct s_light
{
	t_vec3				pos;
	float				ratio;
	t_color				color;
}						t_light;

typedef struct s_ray
{
	t_vec3				o;
	t_vec3				dir;
	float				t;
}						t_ray;

typedef struct s_scene
{
	t_camera			camera;
	t_ambient			ambient;
	t_viewport			viewport;
	t_canvas			canvas;
	t_light				*lights;
	t_object			*objects;
	size_t				obj_count;
	size_t				light_count;
}						t_scene;

/* ----------------------------*/ /*functions*/

t_vec3					vec3_op(enum e_vec3_op op, t_vec3 a, t_vec3 b);
t_vec3					vec3_norm(t_vec3 a);
float					vec3_dot(t_vec3 a, t_vec3 b);
float					vec3_len(t_vec3 a);
t_vec3					vec3_mul(t_vec3 a, float b);

t_vec3					parse_vec3(char *str);
t_color					parse_color(char *str);
t_light					parse_light(char **v);
t_camera				parse_camera(char **v);
t_object				parse_sphere(char **v);
t_object				parse_plane(char **v);
t_object				parse_cylinder(char **v);
void					parse_line(char *line, t_scene *scene);

t_color					color_scale(t_color color, float scale);
t_object				*ray_get_hit(t_scene *scene, t_ray *ray);
int						ray_get_color(t_scene *scene, t_ray *ray);
int						ray_hit_cyl(t_ray *ray, t_cylinder cyl);

t_viewport				viewport_dim(t_canvas canvas, t_camera camera);
t_vec3					viewport_px_pos(t_canvas canvas, t_viewport v, int x,
							int y);

float					cylinder_shading(t_scene *scene, t_ray *ray,
							t_cylinder cylinder);
t_quadratic				solve_quadratic(float a, float b, float c);

#endif
