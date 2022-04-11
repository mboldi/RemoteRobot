import { Component, OnInit } from '@angular/core';
import {RxStompService} from "../../services/rx-stomp.service";
import {FormControl, FormGroup} from "@angular/forms";

@Component({
  templateUrl: './basic-control.component.html',
  styleUrls: ['./basic-control.component.scss']
})
export class BasicControlComponent implements OnInit {
  stepSizeForm = new FormGroup({
    stepSize: new FormControl()
  });

  constructor(private rxStompService: RxStompService) { }

  ngOnInit(): void {
    this.rxStompService.watch('moveArm').subscribe(message => {
      console.log('incoming messeage: ' + message.body)
    })

    this.stepSizeForm.setValue({
      stepSize: 0.1
    });

  }

  moveArm(moveDir: string) {
    const moveVec = this.dirToVector(moveDir);

    this.rxStompService.publish({destination: 'moveArm', body: moveVec});
    console.log(moveVec);
  }

  stopArm() {
    console.log('stop')
  }

  private dirToVector(moveDir: string) {
    const stepSize = this.stepSizeForm.value.stepSize;

    switch (moveDir) {
      case 'up':
        return `(0, ${stepSize}, 0)`;
      case 'down':
        return `(0, -${stepSize}, 0)`;
      case 'left':
        return `(-${stepSize}, 0, 0)`;
      case 'right':
        return `(${stepSize}, 0, 0)`;
    }

    return '(0, 0, 0)';
  }
}
