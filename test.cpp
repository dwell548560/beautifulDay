#include <iostream>
#include <pthread.h>
#include <unistd.h>


#define CONSUMERS_COUNT 2
#define PRODUCERS_COUNT 1

pthread_mutex_t g_mutex;
pthread_cond_t g_cond;
pthread_t g_thread[CONSUMERS_COUNT + PRODUCERS_COUNT];
int share_variable = 0;

void* consumer(void* arg)
{
    int num = (*((int*)arg));

    while(1)
    {
        pthread_mutex_lock(&g_mutex);

        while(share_variable == 0)
        {
            printf("consumer %d begin wait a condition...\n", num);
            pthread_cond_wait(&g_cond, &g_mutex);
        }

        printf("consumer %d end wait a condition...\n", num);
        printf("consumer %d begin wait a condition...\n", num);
        pthread_mutex_unlock(&g_mutex);

        --share_variable;
        sleep(1);

    }


    

}


void* producer(void* arg)
{
     int num = (*((int*)arg));

    while(1)
    {
        pthread_mutex_lock(&g_mutex);

        printf("producer %d begin produce product..\n", num);
        ++share_variable;
        printf("producer %d end produce product..\n", num);
        pthread_cond_signal(&g_cond);
        printf("produce %d notief consumer by condition...\n", num);
        pthread_mutex_unlock(&g_mutex);

        sleep(3);

    }



}








int main(int argc, char** argv)
{
  
    

    for(int i =0; i < CONSUMERS_COUNT;i++ )
    {
        pthread_create(&g_thread[i], NULL, consumer,(void*)&i );
    }


    sleep(1);

    for(int j =0; j < PRODUCERS_COUNT;j++)
    {
        pthread_create(&g_thread[j], NULL, producer,(void*)&j );
    }

    for(int i = 0; i <CONSUMERS_COUNT + PRODUCERS_COUNT; i++  )
    {
        pthread_join(g_thread[i], NULL);
    }

    pthread_mutex_destroy(&g_mutex);
    pthread_cond_destroy(&g_cond);


    return 0;
}