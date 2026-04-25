#version 330

// The camera's position in world-space
uniform vec3 u_cam_pos;

// Color
uniform vec4 u_color;

// Properties of the single point light
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// We also get the uniform texture we want to use.
uniform sampler2D u_texture_1;

// These are the inputs which are the outputs of the vertex shader.
in vec4 v_position;
in vec4 v_normal;

// This is where the final pixel color is output.
// Here, we are only interested in the first 3 dimensions (xyz).
// The 4th entry in this vector is for "alpha blending" which we
// do not require you to know about. For now, just set the alpha
// to 1.
out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 kd = vec3(1.0, 1.0, 1.0);

  vec3 p = v_position.xyz;
  vec3 n = normalize(v_normal.xyz);

  vec3 light_vec = u_light_pos - p;
  float r2 = dot(light_vec, light_vec);
  vec3 l = normalize(light_vec);

  float diffuse_term = max(0.0, dot(n, l));

  vec3 diffuse = kd * (u_light_intensity / r2) * diffuse_term;

  out_color = vec4(diffuse, 1.0);
  
  // (Placeholder code. You will want to replace it.)
  //out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  //out_color.a = 1;
}
