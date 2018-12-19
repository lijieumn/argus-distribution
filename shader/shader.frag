#version 330 core

layout(location=0) out vec4 out_fragcolor;

in vec3 vposition;
in vec2 vtexcoord;
in vec3 vcolor;
in vec3 vnormal;

uniform sampler2D Tex1;
uniform vec3 eyePos;
uniform bool useTex;

//
//	Lights
//
struct DirLight {
	vec3 direction;
	vec3 intensity;
};


//
//	Diffuse color
//
vec3 diffuse(DirLight light, vec3 viewDir, vec3 color, vec3 N){

	if (!gl_FrontFacing) {
		N = -N;
	}


	vec3 L = -1.f*normalize(light.direction);

	// Material values
	vec3 mamb = color;
	vec3 mdiff = color;

	// Color coeffs
	float diff = max( dot(N, L), 0.f );
	vec3 diffuse = 0.5*diff * mdiff * light.intensity;

	vec3 r = reflect(L, N);
	vec3 specular = vec3(0.0);
	if (dot(N, L) > 0.0) {
		specular = 0.7 * pow( max(dot(r, viewDir), 0.0), 10)* color * light.intensity;
	}

	return ( diffuse + specular);
}

void main(){

	DirLight light;
	light.direction = vposition - vec3(0, 0, 10);
	light.intensity = vec3(1, 1, 1);

	DirLight light1;
	light1.direction = vposition - vec3(-10, 0, 10);
	light1.intensity = vec3(0.8, 0.8, 0.8);


	DirLight light2;
	light2.direction = vposition - eyePos;
	light2.intensity = vec3(0.7, 0.7, 0.7);


	DirLight light3;
	light3.direction = vposition - vec3(10, 0, 10);
	light3.intensity = vec3(0.8, 0.8, 0.8);

	vec3 viewDir = normalize(eyePos - vposition);
	vec3 color3;
	if (useTex) {
	    vec4 texColor = texture( Tex1, vtexcoord );
	    color3 = vec3(texColor);
	} else {
	    color3 = vcolor;
	}
	vec3 color = .2*color3;
	color += diffuse(light, viewDir, color3, vnormal);
	color += diffuse(light1, viewDir, color3, vnormal);
	color += diffuse(light2, viewDir, color3, vnormal);
	color += diffuse(light3, viewDir, color3, vnormal);

	out_fragcolor = vec4(color, 1);
}
