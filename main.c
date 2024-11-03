// cylinder.c
#include "./minirt.h"

static float	cylinder_cap_hit(t_ray *ray, t_vec3 cap_c, t_vec3 cap_n,
		float r)
{
	float	denom;
	float	t;
	t_vec3	p;

	denom = vec3_dot(ray->dir, cap_n);
	if (fabs(denom) > 1e-6)
	{
		t = vec3_dot(vec3_op(SUB, cap_c, ray->o), cap_n) / denom;
		if (t < 0)
			return (-1);
		p = vec3_op(ADD, ray->o, vec3_mul(ray->dir, t));
		if (vec3_len(vec3_op(SUB, p, cap_c)) <= r)
			return (t);
	}
	return (-1);
}

static void	cylinder_caps_hit(t_ray *ray, t_cylinder cyl)
{
	float	t_cap;
	float	t_min;

	t_min = -1;
	t_cap = cylinder_cap_hit(ray, cyl.cap1, cyl.normal, cyl.radius);
	if (t_cap >= 0 && t_cap < t_min)
		t_min = t_cap;
	t_cap = cylinder_cap_hit(ray, cyl.cap2, cyl.normal, cyl.radius);
	if (t_cap >= 0 && t_cap < t_min)
		t_min = t_cap;
	if (t_min >= 0)
		ray->t = t_min;
}

static t_quadratic	cylinder_solve_eq(t_ray *ray, t_cylinder cyl)
{
	t_quadratic	q;
	t_vec3		oc;

	oc = vec3_op(SUB, ray->o, cyl.pos);
	q.a = vec3_dot(ray->dir, ray->dir) - pow(vec3_dot(ray->dir, cyl.normal), 2);
	q.b = 2 * (vec3_dot(ray->dir, oc) - vec3_dot(ray->dir, cyl.normal)
			* vec3_dot(oc, cyl.normal));
	q.c = vec3_dot(oc, oc) - pow(vec3_dot(oc, cyl.normal), 2) - pow(cyl.radius,
			2);
	q = solve_quadratic(q.a, q.b, q.c);
	return (q);
}

float	cylinder_shading(t_scene *scene, t_ray *ray, t_cylinder cylinder)
{
	t_vec3	hit_p;
	t_vec3	projected_point_on_axis;
	t_vec3	light_dir;
	float	light_rate;

	hit_p = vec3_op(ADD, ray->o, vec3_mul(ray->dir, ray->t));
	projected_point_on_axis = vec3_op(ADD, cylinder.cap1,
			vec3_mul(cylinder.normal, vec3_dot(vec3_op(SUB, hit_p,
						cylinder.cap1), cylinder.normal)));
	light_dir = vec3_norm(vec3_op(SUB, scene->lights[0].pos, hit_p));
	light_rate = vec3_dot(vec3_norm(vec3_op(SUB, hit_p,
					projected_point_on_axis)), light_dir);
	return (fmax(0, light_rate));
}

int	ray_hit_cyl(t_ray *ray, t_cylinder cyl)
{
	float		t_min;
	float		ts[2];
	int			i;
	t_vec3		p;
	t_quadratic	q;

	t_min = INFINITY;
	q = cylinder_solve_eq(ray, cyl);
	if (q.delta < 0)
		return (0);
	ts[0] = q.t1;
	ts[1] = q.t2;
	i = 0;
	while (i < 2 && ts[i] >= 0)
	{
		p = vec3_op(ADD, ray->o, vec3_mul(ray->dir, ts[i]));
		if (vec3_dot(vec3_op(SUB, p, cyl.cap1),
				cyl.normal) * vec3_dot(vec3_op(SUB, p, cyl.cap2),
				cyl.normal) <= 0 && ts[i] < t_min)
			t_min = ts[i];
		i++;
	}
	ray->t = t_min;
	cylinder_caps_hit(ray, cyl);
	return (ray->t < INFINITY);
}




// intersect.c
t_quadratic	solve_quadratic(float a, float b, float c)
{
	t_quadratic	q;

	q.delta = b * b - 4 * a * c;
	if (q.delta < 0)
		return (q);
	q.t1 = (-b + sqrt(q.delta)) / (2 * a);
	q.t2 = (-b - sqrt(q.delta)) / (2 * a);
	return (q);
}

t_color	color_scale(t_color color, float scale)
{
	t_color	scaled_color;

	scaled_color = (t_color){color.r * scale, color.g * scale, color.b * scale};
	return (scaled_color);
}

static int	ray_hit_sphere(t_ray *ray, t_sphere sphere)
{
	t_vec3		oc;
	t_quadratic	q;

	oc = vec3_op(SUB, ray->o, sphere.pos);
	q.a = vec3_dot(ray->dir, ray->dir);
	q.b = 2.0 * vec3_dot(oc, ray->dir);
	q.c = vec3_dot(oc, oc) - (sphere.radius * sphere.radius);
	q = solve_quadratic(q.a, q.b, q.c);
	if (q.delta < 0 || (q.t1 < 0 && q.t2 < 0))
		return (0);
	else if (q.t1 < 0)
		ray->t = q.t2;
	else if (q.t2 < 0)
		ray->t = q.t1;
	else
		ray->t = fmin(q.t1, q.t2);
	return (1);
}

static int	ray_hit_plane(t_ray *ray, t_plane plane)
{
	float	denom;
	float	t;

	denom = vec3_dot(plane.normal, ray->dir);
	if (fabs(denom) < 0.00001)
		return (0);
	t = vec3_dot(vec3_op(SUB, plane.pos, ray->o), plane.normal) / denom;
	if (t < 0)
		return (0);
	ray->t = t;
	return (1);
}

t_object	*ray_get_hit(t_scene *scene, t_ray *ray)
{
	float	t_min;
	size_t	i;
	int		j;

	t_min = INFINITY;
	i = -1;
	j = -1;
	while (++i < scene->obj_count)
	{
		if (scene->objects[i].type == SPHERE)
			ray_hit_sphere(ray, scene->objects[i].sphere);
		else if (scene->objects[i].type == PLANE)
			ray_hit_plane(ray, scene->objects[i].plane);
		else if (scene->objects[i].type == CYLINDER)
			ray_hit_cyl(ray, scene->objects[i].cylinder);
		if (ray->t < t_min)
		{
			t_min = ray->t;
			j = i;
		}
	}
	ray->t = t_min;
	if (j == -1)
		return (NULL);
	return (&scene->objects[j]);
}


// main.c
void	key_hook(mlx_key_data_t data, void *param)
{
	mlx_t	*mlx;

	mlx = (mlx_t *)param;
	if (data.key == MLX_KEY_ESCAPE)
		mlx_close_window(mlx);
}

void	read_map(t_scene *scene, int fd)
{
	char	*line;

	while (1)
	{
		line = get_next_line(fd);
		if (!line)
			break ;
		parse_line(line, scene);
		free(line);
	}
}

t_vec3	ray_dir(t_scene *scene, int x, int y)
{
	t_vec3	right;
	t_vec3	up;
	float	canvas_px[2];
	t_vec3	px_dir;

	scene->viewport.h = 2.0 * tan((scene->camera.fov * M_PI / 180.0) / 2.0);
	scene->viewport.w = scene->viewport.h * (scene->canvas.w / scene->canvas.h);
	right = vec3_norm(vec3_op(CROSS, vec3_norm(scene->camera.normal),
				(t_vec3){0.0, 1.0, 0.0}));
	up = vec3_norm(vec3_op(CROSS, right, vec3_norm(scene->camera.normal)));
	canvas_px[0] = (x + 0.5f) / scene->canvas.w * 2.0 - 1.0;
	canvas_px[1] = 1.0 - (y + 0.5f) / scene->canvas.h * 2.0;
	px_dir = vec3_op(ADD, vec3_op(ADD, vec3_mul(right, canvas_px[0]
					* scene->viewport.w / 2.0), vec3_mul(up, canvas_px[1]
					* scene->viewport.h / 2.0)),
			vec3_norm(scene->camera.normal));
	return (vec3_norm(px_dir));
}

void	render(mlx_t *mlx, t_scene *scene)
{
	size_t		i;
	size_t		j;
	mlx_image_t	*img;
	t_ray		ray;

	img = mlx_new_image(mlx, scene->canvas.w, scene->canvas.h);
	i = 0;
	scene->viewport = viewport_dim(scene->canvas, scene->camera);
	while (i < scene->canvas.w)
	{
		j = 0;
		while (j < scene->canvas.h)
		{
			ray = (t_ray){scene->camera.pos, ray_dir(scene, i, j), INFINITY};
			mlx_put_pixel(img, i, j, ray_get_color(scene, &ray));
			j++;
		}
		i++;
	}
	mlx_image_to_window(mlx, img, 0, 0);
}

int	main(int argc, char **argv)
{
	int			fd;
	mlx_t		*mlx;
	t_object	objs[10];
	t_light		lights[3];
	t_scene		scene;

	if (argc != 2)
		return (ft_putstr_fd("Invalid number of arguments\n", 2), 1);
	fd = open(argv[1], O_RDONLY);
	if (fd == -1)
		return (ft_putstr_fd("Invalid file\n", 2), 1);
	scene = (t_scene){.objects = objs, .lights = lights,
		.canvas = (t_canvas){1024, 1024}, .obj_count = 0, .light_count = 0};
	mlx = mlx_init(scene.canvas.w, scene.canvas.h, "MiniRT", 1);
	read_map(&scene, fd);
	render(mlx, &scene);
	mlx_key_hook(mlx, key_hook, mlx);
	mlx_loop(mlx);
	mlx_terminate(mlx);
	return (0);
}



// parse.c
t_object	parse_cylinder(char **v)
{
	t_cylinder	cylinder;

	cylinder.pos = parse_vec3(v[1]);
	cylinder.normal = vec3_norm(parse_vec3(v[2]));
	cylinder.radius = ft_atof(v[3]) / 2;
	cylinder.height = ft_atof(v[4]);
	cylinder.color = parse_color(v[5]);
	cylinder.cap1 = vec3_op(SUB, cylinder.pos, vec3_mul(cylinder.normal,
				cylinder.height / 2));
	cylinder.cap2 = vec3_op(ADD, cylinder.pos, vec3_mul(cylinder.normal,
				cylinder.height / 2));
	return ((t_object){.type = CYLINDER, .cylinder = cylinder, .id = rand()});
}

t_object	parse_plane(char **v)
{
	t_plane	plane;

	plane.pos = parse_vec3(v[1]);
	plane.normal = vec3_norm(parse_vec3(v[2]));
	plane.color = parse_color(v[3]);
	return ((t_object){.type = PLANE, .plane = plane, .id = rand()});
}

t_object	parse_sphere(char **v)
{
	t_sphere	sphere;

	sphere.pos = parse_vec3(v[1]);
	sphere.radius = ft_atof(v[2]) / 2;
	sphere.color = parse_color(v[3]);
	return ((t_object){.type = SPHERE, .sphere = sphere, .id = rand()});
}

t_camera	parse_camera(char **v)
{
	t_camera	camera;

	camera.pos = parse_vec3(v[1]);
	camera.normal = vec3_norm(parse_vec3(v[2]));
	camera.fov = ft_atof(v[3]);
	return (camera);
}

t_light	parse_light(char **v)
{
	t_light	light;

	light.pos = parse_vec3(v[1]);
	light.ratio = fmin(ft_atof(v[2]), 1);
	light.color = parse_color(v[3]);
	return (light);
}


// shading.c
static float	sphere_shading(t_scene *scene, t_ray *ray, t_sphere sphere)
{
	t_vec3	normal;
	t_vec3	light_dir;
	float	light_rate;
	t_vec3	hit_p;

	hit_p = vec3_op(ADD, ray->o, vec3_mul(ray->dir, ray->t));
	normal = vec3_norm(vec3_op(SUB, hit_p, sphere.pos));
	light_dir = vec3_norm(vec3_op(SUB, scene->lights[0].pos, hit_p));
	light_rate = vec3_dot(normal, light_dir);
	return (fmax(0, light_rate));
}

static float	plane_shading(t_scene *scene, t_ray *ray, t_plane plane)
{
	t_vec3	normal;
	t_vec3	light_dir;
	float	light_rate;
	t_vec3	hit_p;

	hit_p = vec3_op(ADD, ray->o, vec3_mul(ray->dir, ray->t));
	normal = vec3_norm(plane.normal);
	light_dir = vec3_norm(vec3_op(SUB, scene->lights[0].pos, hit_p));
	light_rate = vec3_dot(normal, light_dir);
	return (fmax(0, light_rate));
}

static int	shadow_shading(t_scene *scene, t_vec3 *hit1, t_vec3 normal,
		t_color color)
{
	t_ray		shadow_ray;
	float		shadow_intensity;
	t_object	*hit2;

	shadow_ray.o = vec3_op(ADD, *hit1, vec3_mul(normal, 0.005));
	shadow_ray.dir = vec3_norm(vec3_op(SUB, scene->lights[0].pos, *hit1));
	shadow_ray.t = INFINITY;
	hit2 = ray_get_hit(scene, &shadow_ray);
	shadow_intensity = 1;
	if (hit2)
	{
		shadow_intensity = shadow_ray.t / vec3_len(vec3_op(SUB,
					scene->lights[0].pos, vec3_op(ADD, shadow_ray.o,
						vec3_mul(shadow_ray.dir, shadow_ray.t))));
		shadow_intensity = fmin(0.999, shadow_intensity);
	}
	color = color_scale(color, shadow_intensity * scene->ambient.ratio);
	return (color.r << 24 | color.g << 16 | color.b << 8 | 0xFF);
}

static t_color	calculate_hit_color(t_scene *scene, t_ray *ray, t_object *hit1)
{
	t_color	color;

	color = (t_color){0, 0, 0};
	if (hit1->type == SPHERE)
	{
		color = hit1->sphere.color;
		color = color_scale(color, sphere_shading(scene, ray, hit1->sphere));
	}
	else if (hit1->type == PLANE)
	{
		color = hit1->plane.color;
		color = color_scale(color, plane_shading(scene, ray, hit1->plane));
	}
	else if (hit1->type == CYLINDER)
	{
		color = hit1->cylinder.color;
		color = color_scale(color, cylinder_shading(scene, ray,
					hit1->cylinder));
	}
	color = color_scale(color, scene->lights[0].ratio);
	return (color);
}

int	ray_get_color(t_scene *scene, t_ray *ray)
{
	t_object	*hit1;
	t_vec3		hit_p;
	t_color		color;
	t_vec3		normal;

	normal = (t_vec3){0, 0, 0};
	hit1 = ray_get_hit(scene, ray);
	if (!hit1)
		return (0);
	hit_p = vec3_op(ADD, ray->o, vec3_mul(ray->dir, ray->t));
	color = calculate_hit_color(scene, ray, hit1);
	if (hit1->type == SPHERE)
		normal = vec3_norm(vec3_op(SUB, hit_p, hit1->sphere.pos));
	else if (hit1->type == PLANE)
		normal = vec3_norm(hit1->plane.normal);
	else if (hit1->type == CYLINDER)
		normal = vec3_norm(vec3_op(SUB, hit_p, hit1->cylinder.cap1));
	return (shadow_shading(scene, &hit_p, normal, color));
}


// utils.c



t_vec3	parse_vec3(char *str)
{
	char	**vec_heap;
	t_vec3	vec;

	vec_heap = ft_split(str, ',');
	if (!vec_heap || ft_split_len(vec_heap) != 3)
	{
		ft_putstr_fd("Error\nInvalid vector format\n", 2);
		exit(1);
	}
	vec.x = ft_atof(vec_heap[0]);
	vec.y = ft_atof(vec_heap[1]);
	vec.z = ft_atof(vec_heap[2]);
	ft_free_split(vec_heap);
	return (vec);
}

t_color	parse_color(char *str)
{
	t_vec3	vec;

	vec = parse_vec3(str);
	return ((t_color){vec.x, vec.y, vec.z});
}

void	parse_line(char *line, t_scene *scene)
{
	char	**s;

	s = ft_split(line, ' ');
	if (!ft_strcmp(s[0], "A"))
		scene->ambient = (t_ambient){fmin(1, ft_atof(s[1])), parse_color(s[2])};
	else if (!ft_strcmp(s[0], "C"))
		scene->camera = parse_camera(s);
	else if (!ft_strcmp(s[0], "L"))
		scene->lights[scene->light_count++] = parse_light(s);
	else if (!ft_strcmp(s[0], "pl"))
		scene->objects[scene->obj_count++] = parse_plane(s);
	else if (!ft_strcmp(s[0], "sp"))
		scene->objects[scene->obj_count++] = parse_sphere(s);
	else if (!ft_strcmp(s[0], "cy"))
		scene->objects[scene->obj_count++] = parse_cylinder(s);
	else
	{
		ft_putstr_fd("Error\nInvalid object type found in the scene\n", 2);
		exit(1);
	}
	ft_free_split(s);
}

t_vec3	viewport_px_pos(t_canvas canvas, t_viewport v, int x, int y)
{
	t_vec3	point;

	point.x = (2 * (x + 0.5) / canvas.w - 1) * v.w;
	point.y = (1 - 2 * (y + 0.5) / canvas.h) * v.h;
	point.z = 1;
	return (point);
}

t_viewport	viewport_dim(t_canvas canvas, t_camera camera)
{
	t_viewport	v;

	v.w = 2 * tan(camera.fov / 2);
	v.h = v.w * (canvas.h / canvas.w);
	return (v);
}



//     vector.c


t_vec3	vec3_op(enum e_vec3_op op, t_vec3 a, t_vec3 b)
{
	if (op == SUB)
		return ((t_vec3){a.x - b.x, a.y - b.y, a.z - b.z});
	else if (op == ADD)
		return ((t_vec3){a.x + b.x, a.y + b.y, a.z + b.z});
	else if (op == MUL)
		return ((t_vec3){a.x * b.x, a.y * b.y, a.z * b.z});
	else if (op == CROSS)
		return ((t_vec3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y
			- a.y * b.x});
	return ((t_vec3){0, 0, 0});
}

t_vec3	vec3_norm(t_vec3 a)
{
	return ((t_vec3){a.x / vec3_len(a), a.y / vec3_len(a), a.z / vec3_len(a)});
}

float	vec3_dot(t_vec3 a, t_vec3 b)
{
	return (a.x * b.x + a.y * b.y + a.z * b.z);
}

float	vec3_len(t_vec3 a)
{
	return (sqrt(a.x * a.x + a.y * a.y + a.z * a.z));
}

t_vec3	vec3_mul(t_vec3 a, float b)
{
	return ((t_vec3){a.x * b, a.y * b, a.z * b});
}




    
