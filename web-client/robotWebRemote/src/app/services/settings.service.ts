import {Injectable} from '@angular/core';
import {myRxStompConfig} from "../my-rx-stomp.config";
import {RxStompService} from "./rx-stomp.service";

@Injectable({
  providedIn: 'root'
})
export class SettingsService {
  private rabbitUrl = 'localhost';

  constructor(private rxStompService: RxStompService) {
  }

  setRabbitUrl(url: string) {
    this.rabbitUrl = url;
    myRxStompConfig.brokerURL = `ws://${url}:15674/ws`;
    this.rxStompService.configure(myRxStompConfig);
  }

  getRabbitUrl(): string {
    return this.rabbitUrl
  }
}
