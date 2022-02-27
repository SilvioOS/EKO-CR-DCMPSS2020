/*--------------------------- ABC ---------------------------------*
 *
 * Author: Leonardo Gabrielli <l.gabrielli@univpm.it>
 * License: GPLv3
 *
 * For a detailed guide of the code and functions see the book:
 * "Developing Virtual Synthesizers with VCV Rack" by L.Gabrielli
 *
 * Copyright 2020, Leonardo Gabrielli
 *
 *-----------------------------------------------------------------*/
#include "EKO-CR.hpp"
#include "SVF.hpp"
#include "RCFilter.hpp"


struct AClave_Eko : Module {
/*	Come parametri io vorrò un knob per la frequenza ed uno per lo shift in frequenza
*	In più l'input lo prendo da un bottone e da un ingresso che funge da gate*/
	enum ParamIds {
		TRIGG,
		PARAM_CUTOFF,
		PARAM_DAMP,
		PARAM_SHIFT,
		NUM_PARAMS,
	};

	enum InputIds {
		GATE_IN,
		NUM_INPUTS,
	};

	enum OutputIds {
		MAIN_OUT,
		//RAMP_OUT,      per visionare la rampa
		NUM_OUTPUTS,
	};

	enum LightsIds {
		NUM_LIGHTS,
	};

/*	La frequenza a cui è accordato il clave è D7, come si può vedere dall'analisi in frequenza del segnale*/
	float tune=2349;

/*	Utilizzando cftool su matlab si può individuare la curva esponenziale che meglio approssima l'inviluppo del
*   clave nella forma: a*exp(-b*x). Nel nostro caso b=0.009551*/

	//MA io non posso dividere semplicemente per la frequenza, perché sono in tempo discreto con Fs=44100!!
	//float b=0.01072;
	float b=0.009551;

/*  Iniziallizzo gli oggetti che verranno utilizzati:
 * 	trig-> rileva quando viene premuto i bottone
 * 	gate-> rileva quando in ingresso al gate arriva un impulso
 * 	pulseGen-> una volta rilevato o il trigger o il gate in ingresso viene generata una delta di dirac
 * 	SVF-> filtro SVF che filtrerà la delta di dirac
 * 	rcf-> filtroRC che viene perturbato in base allo shift in frequenza impostato sull'apposito knob
 * 	quando viene generata una delta di dirac(riga 108). Tale filtro insegue il valore 1 (si veda riga 113)*/
	dsp::BooleanTrigger trig;
	dsp::BooleanTrigger gate;
	dsp::PulseGenerator pulseGen;

	SVF<float> * filt = new SVF<float>(tune, b);
	float hpf, bpf, lpf;

	RCFilter<float> * rcf = new RCFilter<float>();

	AClave_Eko() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

		configParam(PARAM_CUTOFF, 100.f, 20000.f, tune, "Cutoff","Hz");
		configParam(PARAM_DAMP, 0.0001, 0.1, b, "Damping","");
		configParam(PARAM_SHIFT, -12.f, 12.f, 0.f, "Frequency shift","Semitones");
		configButton(TRIGG, "Trigger");

		hpf = bpf = lpf = 0.f;
	}

	void process(const ProcessArgs &args) override;

};

void AClave_Eko::process(const ProcessArgs &args) {

/*	Controlla se qualcuno preme il bottone o se arriva in ingresso un gate, nel caso si procede alla generazione
* 	di un impulso della durata di un campione*/
	bool tap = (params[TRIGG].getValue() > 0.f);
	bool in = (inputs[GATE_IN].getVoltage() > 0.f);
		if (trig.process(tap)||gate.process(in)) {
			pulseGen.trigger(args.sampleTime);
			trig.reset();
		}
	bool pulse = pulseGen.process(args.sampleTime);

	float dirac = (pulse ? 10.f : 0.f);

/*	tau=RC è lo stesso preso nel modello del filtro SVF in quanto si prende come inviluppo l'esponenziale complesso
 *  con lo stesso decadimento*/
	rcf->setTau(params[PARAM_CUTOFF].getValue());

/*	Quando inizio la generazione del suono (ossia ho un nuovo pulse) setto lo stato del filtro ad un valore determinato
 * 	dallo shift in frequenza letto dal knob, dopo di che il filtro insegue il valore 1 con un decadimento pari a quello
 * 	impostato precedentemente*/
	if(pulse){
			rcf->reset(pow(2.f,params[PARAM_SHIFT].getValue()/12.f));
			//lpf=10.f;
		}
	float ramp=rcf->process(1.f);
	float fcs = params[PARAM_CUTOFF].getValue()*ramp;

/*	Anche il filtro SVF segue un decadimento di tipo esponenziale nella forma a*exp(-b*x).
 * 	Il dumping factor df dipende dalla relazione b=df*wn con wn=2*pi*(fc/fs)
 * 	dove fc/fs è la frequenza di cutoff normalizzata
 * 	*/
	float df=params[PARAM_DAMP].getValue()/(3.14*2*(tune/args.sampleRate));

 	filt->setCoeffs(fcs, df);
	filt->process(dirac, &hpf, &bpf, &lpf);

	outputs[MAIN_OUT].setVoltage(lpf);
	//outputs[RAMP_OUT].setVoltage(ramp);
}



struct AClave_EkoWidget : ModuleWidget {
	AClave_EkoWidget(AClave_Eko * module) {

		setModule(module);
		setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/ATemplate.svg")));
		box.size = Vec(6*RACK_GRID_WIDTH, RACK_GRID_HEIGHT);

		{
			ATitle * title = new ATitle(box.size.x);
			title->setText("AClave by Eko");
			addChild(title);
		}

		{
			ATextLabel * title = new ATextLabel(Vec(15, 35));
			title->setText("TRIGGER");
			addChild(title);
		}
		{
			ATextLabel * title = new ATextLabel(Vec(25, 80));
			title->setText("SHIFT");
			addChild(title);
		}

		{
			ATextLabel * title = new ATextLabel(Vec(7, 270));
			title->setText("GATE");
			addChild(title);
		}
		{
			ATextLabel * title = new ATextLabel(Vec(20, 140));
			title->setText("CUTOFF");
			addChild(title);
		}
		{
			ATextLabel * title = new ATextLabel(Vec(27, 200));
			title->setText("DAMP");
			addChild(title);
		}
		{
			ATextLabel * title = new ATextLabel(Vec(55, 270));
			title->setText("OUT");
			addChild(title);
		}

		addInput(createInput<PJ301MPort>(Vec(10, 300), module, AClave_Eko::GATE_IN));

		//addOutput(createOutput<PJ301MPort>(Vec(55, 230), module, AClave_Eko::RAMP_OUT));
		addOutput(createOutput<PJ301MPort>(Vec(55, 300), module, AClave_Eko::MAIN_OUT));

		addParam(createParam<RoundBlackKnob>(Vec(30, 170), module, AClave_Eko::PARAM_CUTOFF));
		addParam(createParam<RoundBlackSnapKnob>(Vec(30, 115), module, AClave_Eko::PARAM_SHIFT));
		addParam(createParam<RoundBlackKnob>(Vec(30, 230), module, AClave_Eko::PARAM_DAMP));
		addParam(createParam<VCVButton>(Vec(35, 70), module, AClave_Eko::TRIGG));


	}

};

/*!!! Ricorda che alla fine devi andare su ABC:cpp e ABC.hpp per aggiungere il module e module widget*/
Model *modelEkoClave = createModel<AClave_Eko, AClave_EkoWidget>("AClave_Eko");
