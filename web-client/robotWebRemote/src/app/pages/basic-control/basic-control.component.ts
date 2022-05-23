import {Component, OnInit, ViewChild} from '@angular/core';
import {RxStompService} from "../../services/rx-stomp.service";
import {FormControl, FormGroup} from "@angular/forms";
declare let RTCPeerConnection: any;

@Component({
  templateUrl: './basic-control.component.html',
  styleUrls: ['./basic-control.component.scss']
})
export class BasicControlComponent implements OnInit {
  videoActive: boolean = false;
  pc: any;
  senderId: string = '';

  webrtcSignalingQueue = 'webrtc'
  webrtcSignalingRespQueue = 'webrtc-resp'

  @ViewChild("remote") remote: any;

  stepSizeForm = new FormGroup({
    stepSize: new FormControl()
  });

  constructor(private rxStompService: RxStompService) { }

  ngOnInit(): void {
    this.rxStompService.watch('robot-status').subscribe(message => {
      console.log('incoming message: ' + message.body)
    })


    this.stepSizeForm.setValue({
      stepSize: 0.1
    });

    this.setupWebRtc();
  }

  public ngOnDestroy() {
    this.pc.close();
    this.videoActive = false;
  }

  moveArm(moveDir: string) {
    let moveBody = moveDir;

    if(moveDir !== 'home') {
      moveBody = this.dirToVector(moveDir);
    }

    this.rxStompService.publish({destination: 'move-robot', body: moveBody});
    console.log(moveBody);
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

  showRemote() {
    try {
      this.pc.createOffer()
        .then((offer: any) => this.pc.setLocalDescription(offer))
        .then(() => {
          this.sendMessage(this.senderId, JSON.stringify({ sdp: this.pc.localDescription }));
          this.videoActive = true;
        });
    } catch (error) {
      this.setupWebRtc();
      console.log(error);
    }
  }

  setupWebRtc() {
    this.senderId = this.guid();

    this.rxStompService.watch(this.webrtcSignalingQueue).subscribe(message => {
      console.log('incoming webrtc signaling msg: ' + message.body);
      this.readMessage(message.body);
    })

    try {
      this.pc = new RTCPeerConnection({
        iceServers: [
          { urls: "stun:stun.services.mozilla.com" },
          { urls: "stun:stun.l.google.com:19302" }
        ]
      }, { optional: [] });
    } catch (error) {
      console.log(error);
      this.pc = new RTCPeerConnection({
        iceServers: [
          { urls: "stun:stun.services.mozilla.com" },
          { urls: "stun:stun.l.google.com:19302" }
        ]
      }, { optional: [] });
    }


    this.pc.onicecandidate = (event: any) => {
      event.candidate ? this.sendMessage(this.senderId, `{"type": "candidate", "candidate": "${event.candidate.candidate}", "id": "${event.candidate.id}", "label": "${event.candidate.label}"}` ) : console.log("Sent All Ice");
    }

    this.pc.onremovestream = (event: any) => {
      console.log('Stream Ended');
    }

    this.pc.ontrack = (event: any) =>
      (this.remote.nativeElement.srcObject = event.streams[0]); // use ontrack
  }

  endVideo() {
    this.pc.close();
    this.videoActive = false;
  }

  sendMessage(senderId: string, data: string) {
    this.rxStompService.publish({destination: this.webrtcSignalingRespQueue, body: data});
    console.log('msg sent: ' + data)
  }

  readMessage(data: any) {
    if (!data) return;
    try {
      const msg = {'sdp': data, 'type': 'offer'};

      this.videoActive = true;
      this.pc.setRemoteDescription(new RTCSessionDescription(msg as RTCSessionDescriptionInit))
        .then(() => this.pc.createAnswer())
        .then((answer: any) => this.pc.setLocalDescription(answer))
        .then(() => this.sendMessage(this.senderId, `{"type": "${this.pc.localDescription.type}", "sdp": "${this.pc.localDescription.sdp}"}`));
    } catch (error) {
      console.log(error);
    }
  }

  guid() {
    return (this.s4() + this.s4() + "-" + this.s4() + "-" + this.s4() + "-" + this.s4() + "-" + this.s4() + this.s4() + this.s4());
  }

  s4() {
    return Math.floor((1 + Math.random()) * 0x10000).toString(16).substring(1);
  }
}
