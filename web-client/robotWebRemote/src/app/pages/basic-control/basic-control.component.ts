import { Component, OnInit } from '@angular/core';
import {RabbitmqService} from "../../services/rabbitmq.service";
import {RxStompService} from "../../services/rx-stomp.service";

@Component({
  templateUrl: './basic-control.component.html',
  styleUrls: ['./basic-control.component.scss']
})
export class BasicControlComponent implements OnInit {

  constructor(private controlService: RabbitmqService,
              private rxStompService: RxStompService) { }

  ngOnInit(): void {
    this.rxStompService.watch('moveArm').subscribe(message => {
      console.log('incoming messeage: ' + message.body)
    })

  }

  moveArm(moveDir: string) {
    this.rxStompService.publish({destination: 'moveArm', body: moveDir})
    console.log(moveDir)
  }

  stopArm() {
    console.log('stop')
  }
}
