#include <stdio.h>
#include <stdlib.h>
#include "portaudio.h"

typedef struct {
    int          frameIndex;
    int          maxFrameIndex;
    float        *recordedSamples;
} PaTestData;

static int recordCallback(const void *inputBuffer, void *outputBuffer,
                        unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo* timeInfo,
                        PaStreamCallbackFlags statusFlags,
                        void *userData)
{
    PaTestData *data = (PaTestData*)userData;
    const float *rptr = (const float*)inputBuffer;
    float *wptr = &data->recordedSamples[data->frameIndex * 1];
    unsigned long framesLeft = data->maxFrameIndex - data->frameIndex;
    long framesToCalc, i;
    int finished;

    if(framesLeft < framesPerBuffer) {
        framesToCalc = framesLeft;
        finished = paComplete;
    }
    else {
        framesToCalc = framesPerBuffer;
        finished = paContinue;
    }
    if(inputBuffer == NULL) {
        for(i = 0; i < framesToCalc; i++) {
            *wptr++ = (0.0f);
        }
    }
    else {
        for(i = 0; i < framesToCalc; i++) {
            *wptr++ = *rptr++;
        }
    }
    data->frameIndex += framesToCalc;
    return finished;
}

int main() {
    Pa_Initialize();

    PaStream *stream;
    PaTestData data;

    int totalFrames, numSamples, numBytes;
    data.maxFrameIndex = totalFrames = 10 * 44100;
    data.frameIndex = 0;
    numSamples = totalFrames * 1;
    numBytes = numSamples * sizeof(float);
    data.recordedSamples = (float*)malloc(numBytes);

    Pa_OpenDefaultStream(&stream, Pa_GetDefaultInputDevice(), 0, paFloat32, 44100, 256, recordCallback, &data);
    Pa_StartStream(stream);

    while(Pa_IsStreamActive(stream)) {
        printf("recording. . .\n");
        Pa_Sleep(1000);
    }

    FILE *sample;
    sample = fopen("recorded.raw", "wb");
    fwrite(data.recordedSamples, 1 * sizeof(float), totalFrames, sample);
    fclose(sample);
    printf("wrote data to 'recorded.raw'\n");

    return 0;
}
