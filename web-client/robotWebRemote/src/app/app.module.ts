import {NgModule} from '@angular/core';
import {BrowserModule} from '@angular/platform-browser';

import {AppRoutingModule} from './app-routing.module';
import {AppComponent} from './app.component';
import {BasicControlComponent} from './pages/basic-control/basic-control.component';
import {BasicControlPanelComponent} from './components/basic-control-panel/basic-control-panel.component';
import {SettingsComponent} from './pages/settings/settings.component';
import {NavbarComponent} from './components/navbar/navbar.component';
import {BrowserAnimationsModule} from '@angular/platform-browser/animations';
import {MatToolbarModule} from "@angular/material/toolbar";
import {MatIconModule} from "@angular/material/icon";
import {MatFormFieldModule} from '@angular/material/form-field';
import {MatButtonModule} from "@angular/material/button";
import {HomeComponent} from './pages/home/home.component';
import {MatInputModule} from "@angular/material/input";
import {MatCardModule} from "@angular/material/card";
import {FormsModule, ReactiveFormsModule} from "@angular/forms";
import {MatGridListModule} from "@angular/material/grid-list";

import {RxStompService} from "./services/rx-stomp.service"
import {rxStompServiceFactory} from "./services/rx-stomp-service-factory";
import { VideoReceiverComponent } from './components/video-receiver/video-receiver.component';

@NgModule({
  declarations: [
    AppComponent,
    BasicControlComponent,
    BasicControlPanelComponent,
    SettingsComponent,
    NavbarComponent,
    HomeComponent,
    VideoReceiverComponent,
  ],
  imports: [
    BrowserModule,
    AppRoutingModule,
    BrowserAnimationsModule,
    MatToolbarModule,
    MatIconModule,
    MatButtonModule,
    MatInputModule,
    MatFormFieldModule,
    MatCardModule,
    ReactiveFormsModule,
    MatGridListModule,
    FormsModule
  ],
  providers: [
    {
      provide: RxStompService,
      useFactory: rxStompServiceFactory
    }
  ],
  bootstrap: [AppComponent]
})
export class AppModule {
}
