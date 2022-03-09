import { Component, OnInit } from '@angular/core';

@Component({
  templateUrl: './basic-control.component.html',
  styleUrls: ['./basic-control.component.scss']
})
export class BasicControlComponent implements OnInit {

  constructor() { }

  ngOnInit(): void {
  }

  moveArm(moveDir: string) {
    console.log(moveDir)
  }

  stopArm() {
    console.log('stop')
  }
}
