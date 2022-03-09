import { ComponentFixture, TestBed } from '@angular/core/testing';

import { BasicControlComponent } from './basic-control.component';

describe('BasicControlComponent', () => {
  let component: BasicControlComponent;
  let fixture: ComponentFixture<BasicControlComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ BasicControlComponent ]
    })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(BasicControlComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
