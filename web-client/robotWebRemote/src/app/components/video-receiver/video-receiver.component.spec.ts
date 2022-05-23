import { ComponentFixture, TestBed } from '@angular/core/testing';

import { VideoReceiverComponent } from './video-receiver.component';

describe('VideoReceiverComponent', () => {
  let component: VideoReceiverComponent;
  let fixture: ComponentFixture<VideoReceiverComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ VideoReceiverComponent ]
    })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(VideoReceiverComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
