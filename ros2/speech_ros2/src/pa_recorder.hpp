#ifndef pa_recorder
#define pa_recorder

#include <portaudio.h>
#include <stdlib.h>
#include <stdint.h>
#include <iostream>

#include "transcription_api.hpp"

#define SAMPLE_SILENCE  (0) // (0.0f) for float
#define PA_SAMPLE_TYPE  paInt16
#define C_SAMPLE_TYPE   int16_t
#define BITS_PER_SAMPLE 16
#define SAMPLE_RATE     16000
#define BUFFER_SIZE     512
#define NUM_CHANNELS    1
#define AUDIO_FORMAT    1
#define NUM_SECONDS     5

typedef struct {
    uint8_t     chunkID[4]      = {'R', 'I', 'F', 'F'};
    uint32_t    chunkSize;      // size of audio + sizeof(WAV_HEADER) - 8
    uint8_t     format[4]       = {'W', 'A', 'V', 'E'};
    uint8_t     subchunk1ID[4]  = {'f', 'm', 't', ' '};
    uint32_t    subchunk1Size   = 16;
    uint16_t    audioFormat     = AUDIO_FORMAT;
    uint16_t    numChannels     = NUM_CHANNELS;
    uint32_t    sampleRate      = SAMPLE_RATE;
    uint32_t    byteRate        = (SAMPLE_RATE * BITS_PER_SAMPLE * NUM_CHANNELS) / 8;
    uint16_t    blockAlign      = (BITS_PER_SAMPLE * NUM_CHANNELS) / 8;
    uint16_t    bitsPerSample   = BITS_PER_SAMPLE;
    uint8_t     subchunk2ID[4]  = {'d', 'a', 't', 'a'};
    uint32_t    subchunk2Size;  // size of audio + sizeof(WAV_HEADER) - 44
} WAV_HEADER;

typedef struct {
    int             frameIndex = 0;
    int             maxFrameIndex;
    C_SAMPLE_TYPE   *recordedSamples;
} PaTestData;

static int recordCallback(const void* inputBuffer, void* outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo* timeInfo,
    PaStreamCallbackFlags statusFlags,
    void* userData)
{
    PaTestData* data = (PaTestData*)userData;
    const C_SAMPLE_TYPE* rptr = (const C_SAMPLE_TYPE*)inputBuffer;
    C_SAMPLE_TYPE* wptr = &data->recordedSamples[data->frameIndex];
    unsigned long framesLeft = data->maxFrameIndex - data->frameIndex;
    long framesToCalc, i;
    int finished;

    if (framesLeft < framesPerBuffer) {
        framesToCalc = framesLeft;
        finished = paComplete;
    }
    else {
        framesToCalc = framesPerBuffer;
        finished = paContinue;
    }
    if (inputBuffer == NULL) {
        for (i = 0; i < framesToCalc; i++) {
            *wptr++ = SAMPLE_SILENCE;
            if (NUM_CHANNELS == 2) { *wptr++ = SAMPLE_SILENCE; }
        }
    }
    else {
        for (i = 0; i < framesToCalc; i++) {
            *wptr++ = *rptr++;
            if (NUM_CHANNELS == 2) { *wptr++ = *rptr++; }
        }
    }
    data->frameIndex += framesToCalc;
    return finished;
}

class paRecorder {
private:
    PaStream* stream;
    PaTestData data;
public:
    char* buffer;
    int size;
    paRecorder() {
        PaStreamParameters inputParameters;
        WAV_HEADER wav;
        int numBytes;

        data.maxFrameIndex = NUM_SECONDS * SAMPLE_RATE * NUM_CHANNELS;
        numBytes = data.maxFrameIndex * sizeof(C_SAMPLE_TYPE);
        data.recordedSamples = (C_SAMPLE_TYPE*)malloc(numBytes);
        size = sizeof(wav) + numBytes;

        wav.chunkSize = numBytes + sizeof(WAV_HEADER) - 8;
        wav.subchunk2Size = numBytes + sizeof(WAV_HEADER) - 44;

        buffer = new char[sizeof(wav) + numBytes];
        memcpy(buffer, &wav, sizeof(wav));

        inputParameters.device = Pa_GetDefaultInputDevice();
        inputParameters.channelCount = NUM_CHANNELS;
        inputParameters.sampleFormat = PA_SAMPLE_TYPE;
        inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
        inputParameters.hostApiSpecificStreamInfo = NULL;

        Pa_OpenStream(&stream, &inputParameters, NULL, SAMPLE_RATE, BUFFER_SIZE, paClipOff, recordCallback, &data);
    }

    void record() {
        data.frameIndex = 0;

        Pa_StartStream(stream);
        while (Pa_IsStreamActive(stream)) {
            std::cout << "recording. . .\n";
            Pa_Sleep(1000);
        }
        memcpy(&buffer[sizeof(WAV_HEADER)], data.recordedSamples, size);
        Pa_StopStream(stream);
    }

    ~paRecorder() {
        free(data.recordedSamples);
        free(buffer);
    }
};

#endif // pa_recorder
