varying vec4 color;
uniform sampler2D reflectMap;

void main(void)
{
    //vec2 ndc = (clipSpace.xy / clipSpace.w);
    //vec2 reflectTexCoords = vec2(ndc.x, -ndc.y);

    //vec4 reflectColor = texture(reflectionTexture, reflectTexCoords);
    // out_Color = reflectColor;
    //gl_FragColor = vec4(1, 0, 1, 1.0);

    vec2 ndc = (color.xy / color.w) / 2.0 + 0.5;

    vec2 reflectTexCoords = vec2(ndc.x, -ndc.y);

    vec4 reflectionColor = texture(reflectMap, reflectTexCoods);
    gl_FragColor = reflectionColor;
}
