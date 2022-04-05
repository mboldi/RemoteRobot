import { Component, OnInit } from '@angular/core';
import {RabbitmqService} from "../../services/rabbitmq.service";

@Component({
  templateUrl: './basic-control.component.html',
  styleUrls: ['./basic-control.component.scss']
})
export class BasicControlComponent implements OnInit {

  constructor(private controlService: RabbitmqService) { }

  ngOnInit(): void {
  }

  moveArm(moveDir: string) {
    console.log(moveDir)
  }

  stopArm() {
    console.log('stop')
  }
}
