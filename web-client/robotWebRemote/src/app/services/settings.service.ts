import {Injectable} from '@angular/core';

@Injectable({
  providedIn: 'root'
})
export class SettingsService {
  private rabbitUrl = 'localhost';

  constructor() {
  }

  setRabbitUrl(url: string) {
    this.rabbitUrl = url;
  }

  getRabbitUrl(): string {
    return this.rabbitUrl
  }
}
