#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "portaudio.h"

typedef struct {
    int     frameIndex = 0;
    int     maxFrameIndex;
    float   *recordedSamples;
} PaTestData;

typedef struct {
    uint8_t     chunkID[4] = {'R', 'I', 'F', 'F'};
    uint32_t    chunkSize;
    uint8_t     format[4] = {'W', 'A', 'V', 'E'};
    uint8_t     subchunk1ID[4] = {'f', 'm', 't', ' '};
    uint32_t    subchunk1Size = 16;
    uint16_t    audioFormat = 3;
    uint16_t    numChannels = 1;
    uint32_t    sampleRate = 44100;
    uint32_t    byteRate = 176400;
    uint16_t    blockAlign = 4;
    uint16_t    bitsPerSample = 32;
    uint8_t     subchunk2ID[4] = {'d', 'a', 't', 'a'};
    uint32_t    subchunk2Size;
} WAV_HEADER;

static int recordCallback(const void *inputBuffer, void *outputBuffer,
                        unsigned long framesPerBuffer,
                        const PaStreamCallbackTimeInfo* timeInfo,
                        PaStreamCallbackFlags statusFlags,
                        void *userData)
{
    PaTestData *data = (PaTestData*)userData;
    const float *rptr = (const float*)inputBuffer;
    float *wptr = &data->recordedSamples[data->frameIndex];
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
    PaStreamParameters inputParameters;
    PaStream *stream;
    PaTestData data;
    WAV_HEADER wav;
    int numBytes;
    FILE *sample;

    data.maxFrameIndex = 5 * 44100;
    numBytes = data.maxFrameIndex * sizeof(float);
    data.recordedSamples = (float*)malloc(numBytes);

    wav.chunkSize = numBytes + sizeof(WAV_HEADER) - 8;
    wav.subchunk2Size = numBytes + sizeof(WAV_HEADER) - 44;

    Pa_Initialize();

    inputParameters.device = Pa_GetDefaultInputDevice();
    inputParameters.channelCount = 1;
    inputParameters.sampleFormat = paFloat32;
    inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
    inputParameters.hostApiSpecificStreamInfo = NULL;

    Pa_OpenStream(&stream, &inputParameters, NULL, 44100, 512, paClipOff, recordCallback, &data);
    Pa_StartStream(stream);

    while(Pa_IsStreamActive(stream)) {
        printf("recording. . .\n");
        Pa_Sleep(1000);
    }

    sample = fopen("recording.wav", "wb");
    fwrite(&wav, sizeof(wav), 1, sample);
    fwrite(data.recordedSamples, sizeof(float), data.maxFrameIndex, sample);
    fclose(sample);
    printf("wrote data to 'recording.wav'\n");

    Pa_Terminate();
    free(data.recordedSamples);

    return 0;
}
