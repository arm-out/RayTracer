#version 330 core

in vec4 position; // raw position in the model coord
in vec3 normal;   // raw normal in the model coord

uniform mat4 modelview; // from model coord to eye coord
uniform mat4 view;      // from world coord to eye coord

// Material parameters
uniform vec4 ambient;
uniform vec4 diffuse;
uniform vec4 specular;
uniform vec4 emision;
uniform float shininess;

// Light source parameters
const int maximal_allowed_lights = 10;
uniform bool enablelighting;
uniform int nlights;
uniform vec4 lightpositions[ maximal_allowed_lights ];
uniform vec4 lightcolors[ maximal_allowed_lights ];

// Output the frag color
out vec4 fragColor;


void main (void){
    if (!enablelighting){
        vec3 N = normalize(normal);
        fragColor = vec4(0.5f*N + 0.5f , 1.0f);
    } else {
        vec4 pos = modelview * position;
        vec3 N = normalize( transpose(inverse(mat3(modelview))) * normal);
        //vec3 N = normalize(mat3(modelview) * normal);
        //vec3 N = normalize(normal);
        fragColor = emision;
        
        vec3 view_dir = normalize(-vec3(pos)); // from pos to eye
        for(int i = 0; i < nlights; i++){
            vec4 light_pos = view * lightpositions[i];
            vec4 light_color = lightcolors[i];
            vec3 light_dir = normalize(pos.w * vec3(light_pos) - light_pos.w * vec3(pos));
            float Lambertian = max(dot(light_dir,N),0.0f);
            vec3 half_dir = normalize(light_dir + view_dir);
            float BlinnPhong = pow(max(dot(half_dir,N),0.0f),shininess);
            fragColor += (ambient + diffuse * Lambertian + specular * BlinnPhong) * light_color;
        }
        //fragColor = vec4(float(nlights),1.0f,0.0f,1.0f);
        //fragColor = vec4(0.5f*N + 0.5f , 1.0f);
    }
}
