#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include <time.h>

///PIN DEFINITION///

// Numéro 2, 1ère zone départ à droite, batterie 8,5

//PIN ULTRASON//
#define TRIG_PIN 0
#define ECHO_PIN 1
//PIN MOTEUR//
#define ENA_PIN 7
#define ENB_PIN 15
#define IN1_PIN 17
#define IN2_PIN 16
#define IN3_PIN 14
#define IN4_PIN 8
//PIN ACTIONNEUR//
#define ACTIONNEUR_PIN 19
//PIN SUIVEUR//
#define SUIVEUR1_PIN 26
#define SUIVEUR2_PIN 27
//PIN TIRETTE//
#define TIRETTE_PIN 28
//PIN INTERRUPTEUR//
#define SWITCH_PIN 22
///DEFINITION DES CONSTANTES///

//CONSTANTES CAPTEUR//  
#define MAX_DISTANCE 15 // Maximum distance in cm for the ultrasonic sensor
#define MIN_DISTANCE 6
#define TEMPS_MAX 1000 // Maximum time in ms without detecting the line
#define SEUIL_BLANC 300 // Seuil pour détecter le blanc
//CONSTANTES MOTEUR//
#define MOTEUR_A 0
#define MOTEUR_B 1
#define INIT_SLOW_SPEED_AB 50; // Slow speed for both motors when turning
#define INIT_DELTA_SPEED_AB 25; // Speed difference for both motors when turning
#define INIT_SPEED_GLOBAL 60;
//CONSTANTES AUTRES//
#define DELAY_DEMARRAGE 1000 // Delay for motor startup in ms
#define DELAY_FOCUS 100000 // Delay for focus in ms
#define SEUIL_DELTAT 15000 // Delay for calculating the time without detecting the line
#define STOP_TIME_ZONE 15000 // Delay for stopping the robot in the  zone pour batterie à 8.7 ~ 9.2 V (ok pour ultrasons)
#define SUIVEURS_OFF 50
/// DEFINITION DES VARIABLES ///

//VARIABLES ULTRASON//
absolute_time_t start_time, end_time;
uint64_t start_us, end_us,duration;
float distance;
//VARIABLES MOTEUR//
uint slice_ENA, slice_ENB, slice_actionneur;
uint channel_ENA, channel_ENB, channel_actionneur;
uint pwma;
uint pwmb;
//VARIABLES TIMERS//
struct repeating_timer timerMS, timerSTOP;
int temps_ms=0;
//AUTRES VARIABLES//
int deltaT=0;
bool state = false;
//INTERRUPTEUR CÔTÉ DE LA TABLE
bool bleu = true;
int INIT_SPEED_A; // (GAUCHE) Initial speed for the motors
int INIT_SPEED_B; // (DROITE) Initial speed for the motors


//DEFINITION DES FLAGS//
//FLAG ARRET MOTEUR//
int flagSTOP=0; //Indique que le robot doit s'arrêter
//FLAG ULTRASON//
int flagUS=0; //Indique que la mesure est prise
int flagENUS=0; //Enable la pris de mesure de la distance
//FLAG SUIVEUR//
int flagSUIVEUR1=0; //Si passage sur le suiveur 1
int flagSUIVEUR2=0; //Si passage sur le suiveur 2
//FLAG TIRETTE//
int flagTIRETTE=0; //Si la tirette est tirée passe à 1

float min(float a, float b) {
    return (a < b) ? a : b;
}

float max(float a, float b) {
    return (a > b) ? a : b;
}

/*Fonction qui met le pwm d'un moteur a la valeur donnée*/
void set_pwm(uint num_mot,uint pwm){
    if(num_mot==0)
    {
        pwm_set_chan_level(slice_ENA, channel_ENA, 256*pwm); // Set duty cycle for motor A
        pwm_set_enabled(slice_ENA, true); // Enable PWM for motor A
    }
    else if(num_mot==1)
    {
        pwm_set_chan_level(slice_ENB, channel_ENB, 256*pwm); // Set duty cycle for motor B
        pwm_set_enabled(slice_ENB, true); // Enable PWM for motor B
    }
}
/*Fonction qui met les moteurs vers l'arrière*/
void set_backward(uint num_moteur){
    if(num_moteur==0)
    {
        gpio_put(IN1_PIN, 1); // Set motor A to move backward
        gpio_put(IN2_PIN, 0); // Set motor A to move backward
    }
    else if(num_moteur==1)
    {
        gpio_put(IN3_PIN, 1); // Set motor B to move backward
        gpio_put(IN4_PIN, 0); // Set motor B to move backward
    }
}
/*Fonction qui met les moteurs vers l'avant*/
void set_forward(uint num_moteur){
    if(num_moteur==0)
    {
        gpio_put(IN1_PIN, 0); // Set motor A to move forward
        gpio_put(IN2_PIN, 1); // Set motor A to move forward
    }
    else if(num_moteur==1)
    {
        gpio_put(IN3_PIN, 0); // Set motor B to move forward
        gpio_put(IN4_PIN, 1); // Set motor B to move forward
    }
}
/*Fonction de temporisation /!\BLOQUE LE SYSTEME/!\*/
void tempo_ms(uint32_t ms) {
    absolute_time_t start = get_absolute_time();
    while (absolute_time_diff_us(start, get_absolute_time()) < ms * 1000) {
        tight_loop_contents(); // Yield to other processes
    }
}
/*Fonction qui arette les moteurs*/
void arret_moteur(){
    // set_backward(MOTEUR_A); // Set motor A to move backward
    // set_backward(MOTEUR_B); // Set motor B to move backward
    set_pwm(MOTEUR_A,0);
    set_pwm(MOTEUR_B,0);
    pwm_set_enabled(slice_ENA, false); // Enable PWM for motor A
    pwm_set_enabled(slice_ENB, false); // Enable PWM for motor B
    gpio_put(IN1_PIN, 1); // Set motor A to move forward
    gpio_put(IN2_PIN, 1);
    gpio_put(IN3_PIN, 1); // Set motor B to move forward
    gpio_put(IN4_PIN, 1);

}
/*Fonction qui controle le moteur, vitesse et sens
 U *ne valeur negative fait rouler vers l'arrière
 Une valeur positive fait rouler vers l'avant*/
void control_moteur(int num_moteur, int consigne) {
    if(num_moteur==0)
    {
        if(consigne>=0)
        {
            set_pwm(MOTEUR_A,consigne); // Set speed for motor A
            set_forward(MOTEUR_A); // Set motor A to move forward
        }
        else
        {
            set_pwm(MOTEUR_A,-consigne); // Stop motor A
            set_backward(MOTEUR_A); // Set motor A to move backward
        }
    }
    else if(num_moteur==1)
    {
        if(consigne>=0)
        {
            set_pwm(MOTEUR_B,consigne); // Set speed for motor B
            set_forward(MOTEUR_B); // Set motor B to move forward
        }
        else
        {
            set_pwm(MOTEUR_B,-consigne); // Stop motor B
            set_backward(MOTEUR_B); // Set motor B to move backward
        }
    }
}

/*Fonction qui tourne à droite*/
void tourner_droite(int vitesse) {
    control_moteur(0, vitesse);   // moteur gauche avance
    control_moteur(1, -vitesse);  // moteur droit recule
}
/*Fonction qui tourne à gauche*/
void tourner_gauche(int vitesse) {
    control_moteur(0, -vitesse);   // moteur gauche recule
    control_moteur(1, vitesse);  // moteur droit avance
}
/*Fonction de démarrage des moteurs*/
void demarrage()
{

    set_pwm(MOTEUR_A,pwma);  // Enable motor A
    set_forward(MOTEUR_A); // Set motor A to move forward

    set_pwm(MOTEUR_B,pwmb); // Enable motor B
    set_forward(MOTEUR_B); // Set motor B to move forward

}
/*Fonction que active l'actionneur /!\utilise des tempos/!\*/
void actionneur(){
    pwm_set_chan_level(slice_actionneur, channel_actionneur, 65465/11);
    tempo_ms(2000);
    pwm_set_chan_level(slice_actionneur, channel_actionneur, 65465/20);
    tempo_ms(2000);
}
/*Fonction qui lance la mesure de distance*/
void measure_distance() {

    absolute_time_t start_time, end_time;
    uint64_t duration=0 ;
    float distance=0;
    end_us = 0;
    start_us = 0;
    // Envoie une impulsion de 10 microsecondes sur TRIG_PIN
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);
}
/*Fonction d'interruptions pour TOUS LES GPIOS --> IMPOSSIBLE DE CREER DES FONCTIONS DIFFERENTES, SINON C'EST LE DERNIER CALLBACK INITIALISÉ QUI EST APPELLÉ*/
void irq_GP_16_29(uint gpio,uint32_t events)
{
    switch(gpio){
        case TIRETTE_PIN:

            flagTIRETTE=1;
            break;

        case ECHO_PIN:

            state=true;
            if(events==8)
            {
                start_time=get_absolute_time();
                start_us = to_us_since_boot(start_time);

            }
            if(events==4)
            {
                end_time=get_absolute_time();
                end_us = to_us_since_boot(end_time);
                duration = end_us-start_us;
                distance = (duration*0.0340)/2.0;
            }
            break;

        default:
            state=false;
            break;
    }
}
/*Fonction TIMER qui gènere une interruption tous les 200ms, permet de lancer les mesures ultrason toutes les 200ms*/
bool irq_compteur_ms(struct repeating_timer *t){

    flagENUS=1;
    return true;
}
/*Fonction qui gère l'arret du robot si aucune ligne n'est detecte apres TEMPS_MAX hors de la ligne*/
bool irq_STOP(struct repeating_timer *t) {

    temps_ms+=10;

    if(temps_ms>SEUIL_DELTAT){
        deltaT+=10;
    }
    else{deltaT = 0;}

}
/*Fonction initialisation de l'ultrason*/
void init_ultrasonic_sensor() {
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_set_irq_enabled(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false); // Désactive d'abord
    gpio_set_irq_enabled_with_callback(ECHO_PIN,GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL,true,&irq_GP_16_29);
}
/*Fonction d'initialisation de la tirette ------ PENSER A VERIFIER SI FRONT MONTANT OU DESCENDANT*/
void init_tirette()
{
    gpio_init(TIRETTE_PIN);
    gpio_set_dir(TIRETTE_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(TIRETTE_PIN, GPIO_IRQ_EDGE_RISE, true, &irq_GP_16_29);
}
/*Fonction d'initialisation des TIMERS*/
void init_compteur(){
    add_repeating_timer_ms(100, irq_compteur_ms, NULL, &timerMS);
    add_repeating_timer_ms(10, irq_STOP, NULL, &timerSTOP);
}
/*Fonction d'initialisation des moteurs*/
void init_motor_driver()
{
    gpio_init(ENA_PIN);
    gpio_init(IN1_PIN);
    gpio_init(IN2_PIN);
    gpio_init(ENB_PIN);
    gpio_init(IN3_PIN);
    gpio_init(IN4_PIN);
    gpio_set_function(ENA_PIN, GPIO_FUNC_PWM);
    gpio_set_function(ENB_PIN, GPIO_FUNC_PWM);
    gpio_set_dir(ENA_PIN, GPIO_OUT);
    gpio_set_dir(IN1_PIN, GPIO_OUT);
    gpio_set_dir(IN2_PIN, GPIO_OUT);
    gpio_set_dir(ENB_PIN, GPIO_OUT);
    gpio_set_dir(IN3_PIN, GPIO_OUT);
    gpio_set_dir(IN4_PIN, GPIO_OUT);


    slice_ENA = pwm_gpio_to_slice_num(ENA_PIN); // slice associée à GP7
    channel_ENA = pwm_gpio_to_channel(ENA_PIN);  // channel associée à GP7
    slice_ENB = pwm_gpio_to_slice_num(ENB_PIN); // slice associée à GP15
    channel_ENB = pwm_gpio_to_channel(ENB_PIN);  // channel associée à GP15

    pwm_set_clkdiv_int_frac (slice_ENA, 38, 3); // diviseur de fréquence = 38 + 3/16
    pwm_set_clkdiv_int_frac (slice_ENB, 38, 3); // diviseur de fréquence = 38 + 3/16

    pwm_set_wrap(slice_ENA, 65465); // valeur max du compteur
    pwm_set_wrap(slice_ENB, 65465); // valeur max du compteur

    pwm_set_phase_correct (slice_ENA, false); // mode phase-correct non activé
    pwm_set_phase_correct (slice_ENB, false); // mode phase-correct non activé

    gpio_put(ENA_PIN, 0); // Enable motor A
    gpio_put(ENB_PIN, 0); // Enable motor B
}

// Fonction qui gère l'interrupteur selon le changement de côté
void init_switch()
{
    gpio_init(SWITCH_PIN);
    gpio_set_dir(SWITCH_PIN, GPIO_IN); // définit le GPIO comme entrée

    // À FAIRE
    gpio_pull_up(SWITCH_PIN);  // si l'interrupteur connecte à GND
    // ou
    gpio_pull_down(SWITCH_PIN); // si l’interrupteur connecte à VCC
}

/*Fonction d'initialisation des Suiveurs*/
void init_suiveur()
{
    adc_init();
    gpio_init(SUIVEUR1_PIN);
    gpio_init(SUIVEUR2_PIN);

    adc_gpio_init(SUIVEUR1_PIN);

    adc_gpio_init(SUIVEUR2_PIN);
}
/*Fonction d'initalisation de l'actionneur*/
void init_actionneur()
{
    gpio_init(ACTIONNEUR_PIN);
    gpio_set_function(ACTIONNEUR_PIN, GPIO_FUNC_PWM);
    gpio_set_dir(ACTIONNEUR_PIN, GPIO_OUT);

    slice_actionneur = pwm_gpio_to_slice_num(ACTIONNEUR_PIN); // slice associée à GP18
    channel_actionneur = pwm_gpio_to_channel(ACTIONNEUR_PIN);  // channel associée à GP18
    pwm_set_clkdiv_int_frac (slice_actionneur, 38, 3); // diviseur de fréquence = 38 + 3/16
    pwm_set_wrap(slice_actionneur, 65465); // valeur max du compteur
    pwm_set_phase_correct (slice_actionneur, false); // mode phase-correct non activé
    gpio_put(ACTIONNEUR_PIN, 0); // Enable motor A
    pwm_set_enabled(slice_actionneur, true);

    // pwm_set_chan_level(slice_actionneur, channel_actionneur, 65465/11); // Set initial duty cycle for the actuator
}
/*Fonction qui apelle toutes les initialisations*/
void init_all()
{
    init_actionneur();
    init_ultrasonic_sensor();
    init_motor_driver();
    init_suiveur();
    pwma=INIT_SPEED_A;
    pwmb=INIT_SPEED_B;
    init_compteur();
}


int main()
{
    float progress = 1;

    unsigned char SLOW_SPEED_AB = INIT_SLOW_SPEED_AB;
    unsigned char DELTA_SPEED_AB = INIT_DELTA_SPEED_AB;

    bleu = !gpio_get(SWITCH_PIN);  // si pull-up et bouton connecté à GND, sinon =
    if(bleu)
    {
        INIT_SPEED_A =  130; // (GAUCHE) Initial speed for the motors
        INIT_SPEED_B =  130; // (DROITE) Initial speed for the motors
    }else{
        INIT_SPEED_A =  130; // (GAUCHE) Initial speed for the motors
        INIT_SPEED_B =  130; // (DROITE) Initial speed for the motors
    }

    //INITIALISATION SYSTEME et TIRETTE//
    stdio_init_all();
    cyw43_arch_init();
    init_tirette();

    while(flagTIRETTE==0){
        printf("%d \n", gpio_get(TIRETTE_PIN));
    }
    printf("%d \n", flagTIRETTE);
    tempo_ms(DELAY_DEMARRAGE); // Attendre TEMPS_DEMARRAGE secondes avant de démarrer le robot
    printf("delayfini");

    unsigned char SPEED_A = INIT_SPEED_A;
    unsigned char SPEED_B = INIT_SPEED_B;

    init_all();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0); //Besoin pour allumer la LED après
    demarrage();
    printf("demarrage moteur");


    while(1){
        //Mesure des valeurs analogiques des suiveurs
        adc_select_input(0);
        uint16_t suiveur1_value = adc_read();
        adc_select_input(1);
        uint16_t suiveur2_value = adc_read();

        //Affichage des mesure de distance et des suiveurs
        printf("ADC Value 1: %d, ADC Value 2: %d\n", suiveur1_value, suiveur2_value);
        printf("Distance: %.2f cm\n", distance);
        printf("Progress: %.2f\n",progress);

        //Mesure de la distance
        if(flagENUS){
            measure_distance();
            flagENUS=0;
        }
        //Gestion de l'ultrason et arret du robot si distance < MAX_DISTANCE
        if(distance<MAX_DISTANCE){
            if (distance<MIN_DISTANCE) {
                set_pwm(MOTEUR_A,0); // Stop motor A
                set_pwm(MOTEUR_B,0); // Stop motor B
                progress = 0;
            } else { // MIN_DISTANCE < distance < MAX_DISTANCE
            progress = (distance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
            progress = max(0.0, min(1.0, progress));
            }
        }
        else if((distance>=MAX_DISTANCE)&&(flagSTOP==0)) {
            progress = 1;
        }
        //DELTA_SPEED_AB = progress * INIT_DELTA_SPEED_AB;
        SLOW_SPEED_AB = progress * INIT_SLOW_SPEED_AB;
        SPEED_A = progress * INIT_SPEED_A;
        SPEED_B = progress * INIT_SPEED_B;
        
        set_pwm(MOTEUR_A,SPEED_A);
        set_pwm(MOTEUR_B,SPEED_B);

        
        if (temps_ms > DELAY_DEMARRAGE + SUIVEURS_OFF && flagSTOP == 0 && (flagSUIVEUR1 == 0 || flagSUIVEUR2 == 0)) {
            if (deltaT > TEMPS_MAX) {
                arret_moteur();
                flagSTOP = 1;
                actionneur();
            } else if (temps_ms > DELAY_FOCUS) {

                if (suiveur2_value < SEUIL_BLANC) {
                    set_pwm(MOTEUR_B,0);
                    set_pwm(MOTEUR_A,(SPEED_A/2)+2*DELTA_SPEED_AB);
                    flagSUIVEUR2=1;
                    flagSUIVEUR1=0;
                }

                if(suiveur1_value< SEUIL_BLANC) {
                    set_pwm(MOTEUR_A,0);
                    set_pwm(MOTEUR_B,(SPEED_B/2)+2*DELTA_SPEED_AB);
                    flagSUIVEUR1=1;
                    flagSUIVEUR2=0;
                }
            } else {
                if (suiveur2_value < SEUIL_BLANC) {
                    set_pwm(MOTEUR_B, SLOW_SPEED_AB);
                    set_pwm(MOTEUR_A, SLOW_SPEED_AB + DELTA_SPEED_AB);
                    flagSUIVEUR2 = 1;
                    flagSUIVEUR1 = 0;
                }

                if (suiveur1_value < SEUIL_BLANC) {
                    set_pwm(MOTEUR_A, SLOW_SPEED_AB);
                    set_pwm(MOTEUR_B, SLOW_SPEED_AB + DELTA_SPEED_AB);
                    flagSUIVEUR1 = 1;
                    flagSUIVEUR2 = 0;
                }
            }
        }

        //Si on detecte une ligne blanche on reset la valeur de deltaT (qui controle l'arret du robot après TEMPS_MAX)
        if (suiveur1_value<SEUIL_BLANC ||  suiveur2_value<SEUIL_BLANC)
        {
            deltaT=0;
        }

        if (temps_ms>SEUIL_DELTAT)
        {
            if(bleu){
                tourner_droite(SLOW_SPEED_AB+DELTA_SPEED_AB);
            }
            else{
                tourner_gauche(SLOW_SPEED_AB+DELTA_SPEED_AB);
            }
            arret_moteur();
            flagSTOP=1;
            actionneur();


        }

        //Affichage de la LED (DEBUG)
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
    }
}