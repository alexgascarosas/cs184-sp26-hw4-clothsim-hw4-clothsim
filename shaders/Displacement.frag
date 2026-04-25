#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  //return 0.0;
  return texture(u_texture_2, uv).r;
}

void main() {
  // YOUR CODE HERE
  vec3 p = v_position.xyz;

  vec3 n = normalize(v_normal.xyz);
  vec3 t = normalize(v_tangent.xyz);
  vec3 b = normalize(cross(n, t));

  mat3 TBN = mat3(t, b, n);

  float du = 1.0 / u_texture_2_size.x;
  float dv = 1.0 / u_texture_2_size.y;

  float height_center = h(v_uv);
  float dU = (h(v_uv + vec2(du, 0.0)) - height_center)
             * u_height_scaling * u_normal_scaling;

  float dV = (h(v_uv + vec2(0.0, dv)) - height_center)
             * u_height_scaling * u_normal_scaling;

  vec3 local_normal = normalize(vec3(-dU, -dV, 1.0));
  vec3 displaced_normal = normalize(TBN * local_normal);

  vec3 light_vec = u_light_pos - p;
  float r2 = dot(light_vec, light_vec);
  vec3 l = normalize(light_vec);

  vec3 v = normalize(u_cam_pos - p);
  vec3 half_vec = normalize(l + v);

  vec3 ka = 0.15 * u_color.rgb;
  vec3 kd = u_color.rgb;
  vec3 ks = vec3(0.5);

  vec3 Ia = vec3(1.0);
  float shininess = 64.0;

  float diffuse_term = max(0.0, dot(displaced_normal, l));
  float specular_term = pow(max(0.0, dot(displaced_normal, half_vec)), shininess);

  vec3 light = u_light_intensity / r2;

  vec3 color = ka * Ia
             + kd * light * diffuse_term
             + ks * light * specular_term;

  out_color = vec4(color, 1.0);
  // (Placeholder code. You will want to replace it.)
  //out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  //out_color.a = 1;
}

