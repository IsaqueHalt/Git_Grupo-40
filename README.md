# Sensor de Bem Estar

Visão Geral

O sensor de bem estar será feito pelo grupo 40 como projeto semestral da disciplina de Introdução à Engenharia Elétrica (0323100), e será um módulo que tem 
como base um microcontrolador embarcado ESP32 integrado aos sensores:

- DHT22 (Temperatura e Umidade)
- MQ-135 (Qualidade do Ar)
- KY-037 (Decibéis)
- MPU-6050 (Aceleração e Velocidade angular)

O microcontrolador enviará, por meio de um protocolo MQTT, as informações para o ThingSpeak (Channel ID: 2259214)*, além de guarda-las em um log na nuvem e em um cartão
microSD.
