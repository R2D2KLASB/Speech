#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "portaudio.h"

typedef struct {
    int          frameIndex;
    int          maxFrameIndex;
    float        *recordedSamples;
} PaTestData;

typedef struct WAV_HEADER {
    uint8_t ChunkID[4] = {'R', 'I', 'F', 'F'};
    uint32_t ChunkSize;
    uint8_t Format[4] = {'W', 'A', 'V', 'E'};
    uint8_t Subchunk1ID[4] = {'f', 'm', 't', ' '};
    uint32_t Subchunk1Size = 16;
    uint16_t AudioFormat = 3;
    uint16_t NumChannels = 1;
    uint32_t SampleRate = 44100;
    uint32_t ByteRate = 176400;
    uint16_t BlockAlign = 4;
    uint16_t BitsPerSample = 32;
    uint8_t Subchunk2ID[4] = {'d', 'a', 't', 'a'};
    uint32_t Subchunk2Size;
} wav_hdr;

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
    data.maxFrameIndex = totalFrames = 5 * 44100;
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

    wav_hdr wav;
    wav.ChunkSize = numBytes + sizeof(wav_hdr) - 8;
    wav.Subchunk2Size = numBytes + sizeof(wav_hdr) - 44;

    FILE *sample;
    sample = fopen("recording.wav", "wb");
    fwrite(&wav, sizeof(wav), 1, sample);
    fwrite(data.recordedSamples, 1 * sizeof(float), totalFrames, sample);
    fclose(sample);
    printf("wrote data to 'recording.wav'\n");

    return 0;
}
