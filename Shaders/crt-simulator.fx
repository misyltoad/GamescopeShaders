// HLSL translation of GLSL CRT Simulator Shader (Thanks @xHybred https://github.com/xHybred/crt-beam-simulator-hlsl/blob/main/crt-simulator.hlsl )
// Note: Adjust syntax as needed for your specific HLSL environment (e.g., DirectX or HLSL shading models).

// From Shadertoy https://www.shadertoy.com/view/XfKfWd
// - Improved version coming January 2025
// - See accompanying article https://blurbusters.com/crt
// - To study more about display science & physics, see Research Portal https://blurbusters.com/area51

/*********************************************************************************************************************/
//
//                     Blur Busters CRT Beam Simulator BFI
//                       With Seamless Gamma Correction
//
//         From Blur Busters Area 51 Display Science, Research & Engineering
//                      https://www.blurbusters.com/area51
//
//             The World's First Realtime Blur-Reducing CRT Simulator
//       Best for 60fps on 240-480Hz+ Displays, Still Works on 120Hz+ Displays
//                 Original Version 2022. Publicly Released 2024.
//
// CREDIT: Teamwork of Mark Rejhon @BlurBusters & Timothy Lottes @NOTimothyLottes
// Gamma corrected CRT simulator in a shader using clever formula-by-scanline trick
// (easily can generate LUTs, for other workflows like FPGAs or Javascript)
// - @NOTimothyLottes provided the algorithm for per-pixel BFI (Variable MPRT, higher MPRT for bright pixels)
// - @BlurBusters provided the algorithm for the CRT electron beam (2022, publicly released for first time)
//
// Contact Blur Busters for help integrating this in your product (emulator, fpga, filter, display firmware, video processor)
//
// This new algorithm has multiple breakthroughs:
//
// - Seamless; no banding*!  (*Monitor/OS configuration: SDR=on, HDR=off, ABL=off, APL=off, gamma=2.4)
// - Phosphor fadebehind simulation in rolling scan.
// - Works on LCDs and OLEDs.
// - Variable per-pixel MPRT. Spreads brighter pixels over more refresh cycles than dimmer pixels.
// - No image retention on LCDs or OLEDs.
// - No integer divisor requirement. Recommended but not necessary (e.g. 60fps 144Hz works!)
// - Gain adjustment (less motion blur at lower gain values, by trading off brightness)
// - Realtime (for retro & emulator uses) and slo-mo modes (educational)
// - Great for softer 60Hz motion blur reduction, less eyestrain than classic 60Hz BFI/strobe.
// - Algorithm can be ported to shader and/or emulator and/or FPGA and/or display firmware.
//
// For best real time CRT realism:
//
// - Reasonably fast performing GPU (many integrated GPUs are unable to keep up)
// - Fastest GtG pixel response (A settings-modified OLED looks good with this algorithm)
// - As much Hz per CRT Hz! (960Hz better than 480Hz better than 240Hz)
// - Integer divisors are still better (just not mandatory)
// - Brightest SDR display with linear response (no ABL, no APL), as HDR boost adds banding
//     (unless you can modify the firmware to make it linear brightness during a rolling scan)
//
// *** IMPORTANT ***
// *** DISPLAY REQUIREMENTS ***
//
// - Best for gaming LCD or OLED monitors with fast pixel response.
// - More Hz per simulated CRT Hz is better (240Hz, 480Hz simulates 60Hz tubes more accurately than 120Hz).
// - OLED (SDR mode) looks better than LCD, but still works on LCD
// - May have minor banding with very slow GtG, asymmetric-GtG (VA LCDs), or excessively-overdriven.
// - Designed for sample & hold displays with excess refresh rate (LCDs and OLEDs);
//     Not intended for use with strobed or impulsed displays. Please turn off your displays' BFI/strobing.
//     This is because we need 100% software control of the flicker algorithm to simulate a CRT beam.
//
// SDR MODE RECOMMENDED FOR NOW (Due to predictable gamma compensation math)
//
// - Best results occur on display configured to standard SDR gamma curve and ABL/APL disabled to go 100% bandfree
// - Please set your display gamma to 2.2 or 2.4, turn off ABL/APL in display settings, and set your OLED to SDR mode.  
// - Will NOT work well with some FALD and MiniLED due to backlight lagbehind effects.
// - Need future API access to OLED ABL/ABL algorithm to compensate for OLED ABL/APL windowing interference with algorithm.
// - This code is heavily commented because of the complexity of the algorithm.
//
/*********************************************************************************************************************/
//
// MIT License
// 
// Copyright 2024 Mark Rejhon (@BlurBusters) & Timothy Lottes (@NOTimothyLottes)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the “Software”), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
/*********************************************************************************************************************/

//------------------------------------------------------------------------------------------------
// Constants Definitions

#include "ReShade.fxh"

#define MOTION_SPEED 10.0

uniform int u_Dummy_AlwaysPaint < source = "gamescope_always_paint"; >;

uniform int u_FrameCount < source = "framecount"; >;
uniform int u_RefreshRateInMhz < source = "gamescope_refresh_mhz"; >;

float GetFramesPerHz()
{
    // Work out the frames per hz here.
    return (float)u_RefreshRateInMhz / (float)60000.0f;
    //return 2.4f; // 144hz
    //return 4.0f; // 240hz
}

#define GAMMA 2.4
#define GAIN_VS_BLUR 0.7
#define FPS_DIVISOR 1.0
#define LCD_ANTI_RETENTION true
#define LCD_INVERSION_COMPENSATION_SLEW 0.001
#define SCAN_DIRECTION 1

//-------------------------------------------------------------------------------------------------
// Utility Macros

float3 clampPixel(float3 a) { return clamp(a, float3(0.0f, 0.0f, 0.0f), float3(1.0f, 1.0f, 1.0f)); }

float SelF1(float a, float b, bool p) { return p ? b : a; }

bool IS_INTEGER(float x) { return floor(x) == x; }
bool IS_EVEN_INTEGER(float x) { return IS_INTEGER(x) && IS_INTEGER(x / 2.0f); }

float GetEffectiveFramesPerHz()
{
    return (LCD_ANTI_RETENTION && IS_EVEN_INTEGER(GetFramesPerHz())) 
           ? GetFramesPerHz() + LCD_INVERSION_COMPENSATION_SLEW 
           : GetFramesPerHz();
}

float fmod(float a, float b)
{
    return (a - b * floor(a / b));
}

//-------------------------------------------------------------------------------------------------
// sRGB Encoding and Decoding Functions

float linear2srgb(float c) {
    float3 j = float3(0.0031308 * 12.92, 12.92, 1.0 / GAMMA);
    float2 k = float2(1.055, -0.055);
    return clamp(j.x, c * j.y, pow(c, j.z) * k.x + k.y);
}

float3 linear2srgb(float3 c) {
    return float3(linear2srgb(c.r), linear2srgb(c.g), linear2srgb(c.b));
}

float srgb2linear(float c) {
    float3 j = float3(0.04045, 1.0 / 12.92, GAMMA);
    float2 k = float2(1.0 / 1.055, 0.055 / 1.055);
    return SelF1(c * j.y, pow(c * k.x + k.y, j.z), c > j.x);
}

float3 srgb2linear(float3 c) {
    return float3(srgb2linear(c.r), srgb2linear(c.g), srgb2linear(c.b));
}

//-------------------------------------------------------------------------------------------------
// Gets pixel from the unprocessed framebuffer

float3 getPixelFromOrigFrame(float2 uv, float getFromHzNumber, float currentHzCounter) {
    if ((getFromHzNumber > currentHzCounter) || (getFromHzNumber < currentHzCounter - 2.0f)) {
        return float3(0.0, 0.0, 0.0);
    }

    float shiftAmount = MOTION_SPEED / 1000.0f;
    float baseShift = fmod(getFromHzNumber * shiftAmount, 1.0f);

    float px = 1.0 / ReShade::ScreenSize.x;
    uv.x = fmod(uv.x + baseShift + px * 0.1f, 1.0f) - px * 0.1f;

    return tex2Dlod(ReShade::BackBuffer, float4(uv, 0.0f, 0.0f)).rgb;
}

//-------------------------------------------------------------------------------------------------
// CRT Rolling Scan Simulation With Phosphor Fade

float3 getPixelFromSimulatedCRT(float2 uv, float crtRasterPos, float crtHzCounter, float framesPerHz) {
    float3 pixelPrev2 = srgb2linear(getPixelFromOrigFrame(uv, crtHzCounter - 2.0f, crtHzCounter));
    float3 pixelPrev1 = srgb2linear(getPixelFromOrigFrame(uv, crtHzCounter - 1.0f, crtHzCounter));
    float3 pixelCurr = srgb2linear(getPixelFromOrigFrame(uv, crtHzCounter, crtHzCounter));

    float3 result = float3(0.0f, 0.0f, 0.0f);
    float brightnessScale = framesPerHz * GAIN_VS_BLUR;
    float3 colorPrev2 = pixelPrev2 * brightnessScale;
    float3 colorPrev1 = pixelPrev1 * brightnessScale;
    float3 colorCurr = pixelCurr * brightnessScale;

#if SCAN_DIRECTION == 1
    float tubePos = 1.0 - uv.y;
#elif SCAN_DIRECTION == 2
    float tubePos = uv.y;
#elif SCAN_DIRECTION == 3
    float tubePos = uv.x;
#elif SCAN_DIRECTION == 4
    float tubePos = 1.0 - uv.x;
#endif

    for (int ch = 0; ch < 3; ch++) {
        float Lprev2 = colorPrev2[ch];
        float Lprev1 = colorPrev1[ch];
        float Lcurr = colorCurr[ch];

        if (Lprev2 <= 0.0 && Lprev1 <= 0.0 && Lcurr <= 0.0) {
            result[ch] = 0.0;
            continue;
        }

        float tubeFrame = tubePos * framesPerHz;
        float fStart = crtRasterPos * framesPerHz;
        float fEnd = fStart + 1.0;

        float startPrev2 = tubeFrame - framesPerHz;
        float endPrev2 = startPrev2 + Lprev2;

        float startPrev1 = tubeFrame;
        float endPrev1 = startPrev1 + Lprev1;

        float startCurr = tubeFrame + framesPerHz;
        float endCurr = startCurr + Lcurr;

#define INTERVAL_OVERLAP(Astart, Aend, Bstart, Bend) max(0.0, min(Aend, Bend) - max(Astart, Bstart))
        float overlapPrev2 = INTERVAL_OVERLAP(startPrev2, endPrev2, fStart, fEnd);
        float overlapPrev1 = INTERVAL_OVERLAP(startPrev1, endPrev1, fStart, fEnd);
        float overlapCurr = INTERVAL_OVERLAP(startCurr, endCurr, fStart, fEnd);

        result[ch] = overlapPrev2 + overlapPrev1 + overlapCurr;
    }

    return linear2srgb(result);
}

//-------------------------------------------------------------------------------------------------
// Main Pixel Shader

// TODO: Port to compute so we can run asynchronously from game work in the compositor.

void VS_PostProcess(in uint id : SV_VertexID, out float4 position : SV_Position, out float2 texcoord : TEXCOORD)
{
	texcoord.x = (id == 2) ? 2.0 : 0.0;
	texcoord.y = (id == 1) ? 2.0 : 0.0;
	position = float4(texcoord * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
}

float3 PS_CrtSimulator(float4 vpos : SV_Position, float2 uv : TexCoord) : SV_Target {
    float effectiveFrame = floor(float(u_FrameCount) * FPS_DIVISOR);
    float crtRasterPos = fmod(effectiveFrame, GetEffectiveFramesPerHz()) / GetEffectiveFramesPerHz();
    float crtHzCounter = floor(effectiveFrame / GetEffectiveFramesPerHz());
    
    return getPixelFromSimulatedCRT(uv, crtRasterPos, crtHzCounter, GetEffectiveFramesPerHz());
}

technique CrtSimulator <
ui_tooltip = "BlurBusters CRT simulator";
>
{
    pass
    {
        VertexShader = VS_PostProcess;
        PixelShader = PS_CrtSimulator;
    }
}

//-------------------------------------------------------------------------------------------------
// Credits Reminder:
// Please credit BLUR BUSTERS & TIMOTHY LOTTE if this algorithm is used in your project/product.
// Hundreds of hours of research was done on related work that led to this algorithm.
//-------------------------------------------------------------------------------------------------
