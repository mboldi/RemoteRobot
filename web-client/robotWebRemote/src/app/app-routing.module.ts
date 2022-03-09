import { NgModule } from '@angular/core';
import { RouterModule, Routes } from '@angular/router';
import {BasicControlComponent} from "./pages/basic-control/basic-control.component";
import {SettingsComponent} from "./pages/settings/settings.component";
import {HomeComponent} from "./pages/home/home.component";

const routes: Routes = [
  {
    path: '',
    pathMatch: 'full',
    redirectTo: 'home'
  },
  {
    path: 'home',
    component: HomeComponent
  },
  {
    path: 'basiccontrol',
    component: BasicControlComponent
  },
  {
    path: 'settings',
    component: SettingsComponent
  }
];

@NgModule({
  imports: [RouterModule.forRoot(routes)],
  exports: [RouterModule]
})
export class AppRoutingModule { }
