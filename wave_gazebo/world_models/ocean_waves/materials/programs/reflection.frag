varying vec4 color;

void main(void)
{
    //vec2 ndc = (clipSpace.xy / clipSpace.w);
    //vec2 reflectTexCoords = vec2(ndc.x, -ndc.y);

    //vec4 reflectColor = texture(reflectionTexture, reflectTexCoords);
    // out_Color = reflectColor;
    //gl_FragColor = vec4(1, 0, 1, 1.0);
    gl_FragColor = color;
}
