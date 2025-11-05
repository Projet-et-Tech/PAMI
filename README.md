# README - PAMI

## Objectif

`PAMI.c` contient la logique de contrôle d'un Petit Actionneur Mobile Indépendant (PAMI) sur Raspberry Pi Pico. Il gère :
- les moteurs (deux moteurs A/B),
- un capteur ultrason (TRIG / ECHO),
- deux capteurs suiveurs de ligne (entrées ADC),
- un actionneur (main) piloté en PWM,
- une tirette de démarrage
- un interrupteur de côté.

Le programme utilise le PWM pour piloter les moteurs, des interruptions pour mesurer l'écho ultrason et des timers répétés pour des tâches périodiques. La boucle principale lit les capteurs et applique la logique de déplacement.

## Constantes et seuils clés
- MAX_DISTANCE — distance ultrason utilisée pour ralentir
- MIN_DISTANCE — en dessous, les moteurs sont arrêtés (progress = 0)
- SEUIL_BLANC — seuil pour détecter une ligne blanche
- TEMPS_MAX — durée maximale (ms) hors ligne avant arrêt logique
- DELAY_DEMARRAGE — délai de démarrage après tirage de la tirette (obligatoire en match)
- DELAY_FOCUS — délai long utilisé pour changer de stratégie de suivi

## Fonctions importantes et leur rôle

- set_pwm(num_mot, pwm)
  - Définit le duty-cycle pour le moteur A ou B via `pwm_set_chan_level` et active la tranche PWM.
  - Remarque : la valeur de duty est multipliée par 256 dans le code source. Le wrap PWM est fixé à 65465.

- set_forward / set_backward
  - Pilotent les pins de direction (IN1/IN2 pour A, IN3/IN4 pour B).

- control_moteur(num_moteur, consigne)
  - Contrôle haut niveau du moteur : consigne positive => avant, négative => arrière.

- demarrage()
  - Applique `pwma`/`pwmb` aux moteurs et met le sens avant.

- actionneur()
  - Déplace l'actionneur en changeant le PWM puis en appelant `tempo_ms()` (délai bloquant).

- measure_distance()
  - Génère une impulsion TRIG de 10 µs ; la mesure réelle (durée ECHO) est traitée dans l'IRQ.

- tempo_ms(ms)
  - Attente active utilisant `get_absolute_time()` et `tight_loop_contents()` — bloque l'exécution.

## Comportement de la boucle principale (blocs importants)

1) Attente de la tirette : le code boucle sur `flagTIRETTE` et affiche l'état de la pin jusqu'au démarrage.

2) `tempo_ms(DELAY_DEMARRAGE)` — délai de démarrage bloquant.

3) Appel à `init_all()` puis `demarrage()` pour lancer les moteurs.

4) Boucle : à chaque itération :
- lecture des capteurs (ultrason, suiveurs de ligne),
- ajustement des moteurs selon la distance mesurée,
- logique de suivi de ligne avec temporisation pour perte de ligne,
- gestion de l'actionneur,
