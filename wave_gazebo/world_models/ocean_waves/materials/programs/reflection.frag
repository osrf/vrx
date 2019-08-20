varying vec4 projectionCoord;

uniform sampler2D reflectMap;
odsfafj
dsfasdf
void main(void)
{
  // Do the tex projection manually so we can distort _after_
  vec2 final = projectionCoord.xy / projectionCoord.w;
  vec4 reflectionColour = texture2D(reflectMap, final);

  gl_FragColor = reflectionColour;
}
