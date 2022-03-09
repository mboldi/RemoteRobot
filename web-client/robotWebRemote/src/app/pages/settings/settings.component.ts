import {Component, OnInit} from '@angular/core';
import {FormBuilder, FormGroup, Validators} from "@angular/forms";
import {SettingsService} from "../../services/settings.service";

@Component({
  templateUrl: './settings.component.html',
  styleUrls: ['./settings.component.scss']
})
export class SettingsComponent implements OnInit {
  settingsForm: FormGroup;

  constructor(
    private fb: FormBuilder,
    private settingsService: SettingsService) {

    this.settingsForm = this.fb.group({
      serverIp: [this.settingsService.getRabbitUrl(), Validators.required]
    });


  }

  ngOnInit(): void {
  }

  save() {
    this.settingsService.setRabbitUrl(this.settingsForm.value.serverIp)
    console.log('ip mentve')
  }
}
