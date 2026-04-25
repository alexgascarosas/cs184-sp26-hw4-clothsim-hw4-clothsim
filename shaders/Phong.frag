#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 p = v_position.xyz;
  vec3 n = normalize(v_normal.xyz);

  vec3 light_vec = u_light_pos - p;
  float r2 = dot(light_vec, light_vec);
  vec3 l = normalize(light_vec);

  vec3 v = normalize(u_cam_pos - p);
  vec3 h = normalize(l + v);

  vec3 ka = 0.15 * u_color.rgb;
  vec3 kd = u_color.rgb;
  vec3 ks = vec3(0.7, 0.7, 0.7);

  vec3 Ia = vec3(1.0, 1.0, 1.0);
  float shininess = 64.0;

  float diffuse_term = max(0.0, dot(n, l));
  float specular_term = pow(max(0.0, dot(n, h)), shininess);

  vec3 light = u_light_intensity / r2;

  vec3 color = ka * Ia
             + kd * light * diffuse_term
             + ks * light * specular_term;

  out_color = vec4(color, 1.0);
  
  // (Placeholder code. You will want to replace it.)
  //out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  //out_color.a = 1;
}

