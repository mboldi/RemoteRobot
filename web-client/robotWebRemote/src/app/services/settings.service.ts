import {Injectable} from '@angular/core';
import {myRxStompConfig} from "../my-rx-stomp.config";

@Injectable({
  providedIn: 'root'
})
export class SettingsService {
  private rabbitUrl = 'localhost';

  constructor() {
  }

  setRabbitUrl(url: string) {
    this.rabbitUrl = url;
    myRxStompConfig.brokerURL = `ws://${url}:15674/ws`
  }

  getRabbitUrl(): string {
    return this.rabbitUrl
  }
}
