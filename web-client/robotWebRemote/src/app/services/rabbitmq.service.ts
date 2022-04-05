import { Injectable } from '@angular/core';
import {AMQPWebSocketClient, AMQPQueue} from "@cloudamqp/amqp-client";
import {SettingsService} from "./settings.service";

@Injectable({
  providedIn: 'root'
})
export class RabbitmqService {

  //private amqp: AMQPWebSocketClient;
  //private controlQueue: AMQPQueue;

  constructor(
    private settingsService: SettingsService
  ) {
    /*this.amqp = new AMQPWebSocketClient(`amqp://${settingsService.getRabbitUrl()}:5672`);

    console.log('Connecting...');

    this.amqp.connect().then(value => {
      console.log('Connected')
      console.log(value);
    })*/
  }
}
