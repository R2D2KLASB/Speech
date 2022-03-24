#ifndef transcribe_api
#define transcribe_api

#include <curl/curl.h>
#include <string>
#include <chrono>

using namespace std::chrono;

typedef struct {
    time_point<steady_clock> issueTime;
    int expiryTime = 60000000;
    std::string token;
} TOKEN;

size_t writeCallback(char *contents, size_t size, size_t nmemb, void *userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

void getToken(TOKEN* token) {
    std::string _token;
    struct curl_slist* chunk = NULL;
    token->issueTime = steady_clock::now();
    
    chunk = curl_slist_append(chunk, "Content-type: application/x-www-form-urlencoded");
    chunk = curl_slist_append(chunk, "Content-Length: 0");
    chunk = curl_slist_append(chunk, "Ocp-Apim-Subscription-Key: 162470352a474535bf355e00b6e41870");

    CURL* curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, "https://westeurope.api.cognitive.microsoft.com/sts/v1.0/issueToken");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &_token);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "");

    curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    curl_slist_free_all(chunk);

    token->token = _token;
}

void checkToken(TOKEN* token) {
    time_point<steady_clock> end = steady_clock::now();
    int deltaTime = duration_cast<microseconds>(end - token->issueTime).count();

    if(deltaTime > token->expiryTime) {
        getToken(token);
    }
}

std::string transcribeAudio(TOKEN* token, char* buffer, int size) {
    struct curl_slist* chunk = NULL;
    std::string transcription;

    checkToken(token);
    std::string auth = "Authorization: Bearer " + token->token;

    chunk = curl_slist_append(chunk, auth.c_str());
    chunk = curl_slist_append(chunk, "Content-type: audio/wav; codec=audio/pcm; samplerate=16000");

    CURL* curl = curl_easy_init();
    curl_easy_setopt(curl, CURLOPT_URL, "https://westeurope.stt.speech.microsoft.com/speech/recognition/conversation/cognitiveservices/v1?language=nl-NL");
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &transcription);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, size);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, buffer);

    curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    curl_slist_free_all(chunk);

    return transcription;
}

#endif // transcribe_api
