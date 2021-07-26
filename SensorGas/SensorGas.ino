#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Definicoes do LCD
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
// Verifica se ha novas actualizacoes mno LCD
#define LCD_TEMPO_ENTRE_VERIFICACOES 1000 //ms 

#define ledVerm 7
#define ledVerd 8
int threshold = 350 ;



// Definicoes do Sensor MQ-2
#define pin_Analogica  0//A0

// A cada segundo vai fazer uma nova leitura analogica
#define MQ7_tempo_Entre_Leituras 1000 //ms


/* A tomada do controlo de semafaro vai aguardar ate 100 ticks */

//Tempo para aguardar tomada de controle do semaforo
#define TEMPO_PARA_AGUARDAR_SEMAFORO (TickType_t) 100

//Tempo para aguardar tomada de controle da fila
#define TEMPO_PARA_AGUARDAR_FILA (TickType_t) 100

//  Declaracao das Queues (Filas)
QueueHandle_t queue_sensor_MQ7;


//Declaracao do Semaforo
SemaphoreHandle_t semaforo;



// Prototipando as tarefas

void tarefa_LCD(void * pvParameters);
void tarefa_Sensor_MQ7(void * pvParameters);


void setup() {

  Serial.begin(115200); // Inicializa o serial monitor
  lcd.init();          // Inicializa o LCD 
  lcd.backlight();    // Liga o backlight
  lcd.clear();       // Limpa o LCD


  pinMode(ledVerd,OUTPUT);
  pinMode(ledVerm,OUTPUT);
  //Criacao das Filas
  queue_sensor_MQ7 = xQueueCreate(1, sizeof(int));
  


  // Verificacao se as filas foram criadas correctamente
  
    if((queue_sensor_MQ7 == NULL) ){
      Serial.print("A fila nao pode ser criada");
      while(1)
      {
        
      }
      
    }
    //Criacao um semaforo do tipo Mutex

    semaforo = xSemaphoreCreateMutex();

    // verifica se o semaforo foi creado correctamente
    if(semaforo == NULL){
      Serial.println("O semaforo nao pode ser criado");
      while(1)
      {
        // Caso nao o porgrama para
      }
      
        }

         // Criacao das Tarefas
   /* Tarefa_LCD */
  xTaskCreate(tarefa_LCD, "Tarefa LCD", 156, NULL, 1, NULL);

    /* Tarefa_Sensor_MQ2 */
  xTaskCreate(tarefa_Sensor_MQ7,  "Tarefa Sensor MQ2", 156, NULL, 2, NULL);

    /* tarefa_Sensor_Ultrasonico */
//  xTaskCreate(tarefa_Sensor_ultrasonico, "tarefa Sensor ultrasonico", 156, NULL, 3, NULL);
        }
  void loop()
  {
  }
  
  // Implementacao das funcoes das tarefas

  void tarefa_LCD(void *pvParameters){
   
    int leitura_MQ7 = 0;
    char linha_str[16];

   while(1){

    //Escreva a ultima leitura analogica do sensor MQ-2
    if(xQueuePeek(queue_sensor_MQ7,&leitura_MQ7,TEMPO_PARA_AGUARDAR_FILA)){

      lcd.setCursor(0,0);
      


      sprintf(linha_str, "Sensor value:%d ",leitura_MQ7);
      lcd.print(linha_str); 
      
    }

    /* Aguarda tempo defino em LCD_TEMPO_ENTRE_VERIFICACOES
     para verificar se ha novas leituras a serem escritas no Display
     */

     vTaskDelay(LCD_TEMPO_ENTRE_VERIFICACOES/portTICK_PERIOD_MS);
   }
    
  }

  void tarefa_Sensor_MQ7(void *pvParameters){
    int leitura_Analogica = 0;

    while(1)
    {
      leitura_Analogica=analogRead(pin_Analogica);

      // Insere leitura na Fila
      xQueueOverwrite(queue_sensor_MQ7,(void*)&leitura_Analogica);
      if(leitura_Analogica>threshold){
        digitalWrite(ledVerm, HIGH);
        digitalWrite(ledVerd, LOW);
        lcd.setCursor(1,1);
        lcd.print("Gas detected");
        
        }
      else{
        digitalWrite(ledVerm, LOW);
        digitalWrite(ledVerd, HIGH);
        lcd.clear();
      }
        
      

      /*
       Escreve leitura na serial , Aqui a tentiva de se tomar o controle do semaforo eh 
       feita ate o tempo definido em Tempo-Para-Guardar-Semaforo.
       Caso nao forpossivel tomar o controle do semaforo ate este tempo (pois outra)
       tarefa o esta utilizando, nada eh escrito na serial.
       */ 

       if(xSemaphoreTake(semaforo,TEMPO_PARA_AGUARDAR_SEMAFORO) == pdTRUE)
       {
        Serial.print(" Leitura MQ-7: ");
        Serial.println(leitura_Analogica);
        xSemaphoreGive(semaforo);
       }

       /*
         Aguarda tempo definido em MQ_2_TEMPO_ENTRE_LEITURAS
         para fazer proxima leitura
        */
        vTaskDelay(MQ7_tempo_Entre_Leituras / portTICK_PERIOD_MS);
    }
  }
