function h0 = maxsigmaplot(varargin)
%   SIGMAPLOT  Singular value plot of linear systems.
%
%   SIGMAPLOT, an extension of SIGMA, provides a command line interface for   
%   customizing the plot appearance.
%
%   SIGMAPLOT(SYS) produces a singular value (SV) plot of the frequency
%   response of the dynamic system SYS. The frequency range and number of
%   points are chosen automatically. See BODE for details on the notion of
%   frequency in discrete time.
%
%   SIGMAPLOT(SYS,{WMIN,WMAX}) draws the SV plot for frequencies ranging
%   between WMIN and WMAX (in radian/timeunit, where timeunit is specified
%   in SYS.TimeUnit).
%
%   SIGMAPLOT(SYS,W) uses the user-supplied vector W of frequencies (in
%   radians/timeunit, where timeunit is specified in SYS.TimeUnit), at
%   which the frequency response is to be evaluated. See LOGSPACE to
%   generate logarithmically spaced frequency vectors.
%
%   SIGMAPLOT(SYS,W,TYPE) or SIGMAPLOT(SYS,[],TYPE) draws the following
%   modified SV plots depending on the value of TYPE:
%          TYPE = 1     -->     SV of  inv(SYS)
%          TYPE = 2     -->     SV of  I + SYS
%          TYPE = 3     -->     SV of  I + inv(SYS) 
%   SYS should be a square system when using this syntax.
%
%   SIGMAPLOT(AX,...) plots into the axes with handle AX.
%
%   SIGMAPLOT(..., PLOTOPTIONS) plots the singular values with the options
%   specified in PLOTOPTIONS. See SIGMAOPTIONS for more details. 
%
%   H = SIGMAPLOT(...) returns the handle to the singular value plot. You 
%   can use this handle to customize the plot with the GETOPTIONS and 
%   SETOPTIONS commands.  See SIGMAOPTIONS for a list of available plot 
%   options.
%
%   Example:
%       sys = rss(5);
%       h = sigmaplot(sys);
%       % Change units to Hz 
%       setoptions(h,'FreqUnits','Hz');
%   
%   See also WCSIGMAPLOT, SIGMAOPTIONS, WRFC/SETOPTIONS, WRFC/GETOPTIONS, DYNAMICSYSTEM.

%	Andrew Grace  7-10-90
%	Revised ACWG 6-21-92
%	Revised by Richard Chiang 5-20-92
%	Revised by W.Wang 7-20-92
%       Revised P. Gahinet 5-7-96
%       Revised A. DiVergilio 6-16-00
%       Revised K. Subbarao 10-11-01
%   Copyright 1986-2021 The MathWorks, Inc.

% Get argument names
for ct = length(varargin):-1:1
   ArgNames(ct,1) = {inputname(ct)};
end

% Parse input list
% Check for axes argument
if ishghandle(varargin{1},'axes')
   ax = varargin{1};
   varargin(1) = [];
   ArgNames(1) = [];
else
   ax = [];
end

try
   [sysList,Extras,OptionsObject] = DynamicSystem.parseRespFcnInputs(varargin,ArgNames);
   if ~isempty(OptionsObject) && ~isa(OptionsObject,'plotopts.SigmaOptions')
       error('Controllib:plots:InvalidPlotOptions',...
                getString(message('Controllib:plots:InvalidPlotOptions','sigmaoptions')));
   end
   [sysList,w,type] = DynamicSystem.checkSigmaInputs(sysList,Extras);
   TimeUnits = sysList(1).System.TimeUnit; % first system determines units
   % Warn about and skip empty systems
   isEmptySys = arrayfun(@(x) isempty(x.System),sysList);
   if any(isEmptySys)
      warning(message('Control:analysis:PlotEmptyModel'))
   end
   sysList = sysList(~isEmptySys);
   % Check time unit consistency when specifying w or {wmin,wmax}
   if ~(isempty(w) || ltipack.hasMatchingTimeUnits(TimeUnits,sysList.System))
      error(message('Control:analysis:AmbiguousFreqSpec'))
   end

   % Create plot (visibility ='off')
   if isempty(ax)
      ax = gca;
   end

   % Check to create sigmaplot using control charts
   %    - version should be 2
   %    - parent figure should not be in live editor
   if controllibutils.CSTCustomSettings.getCSTPlotsVersion == 2 && ...
           ~matlab.internal.editor.figure.FigureUtils.isEditorFigure(ancestor(ax,'figure'))
       h = controllib.chart.internal.utils.ltiplot("sigma",ax,...
           SystemData=sysList,Frequency=w,Options=OptionsObject);
       if nargout
           h0 = h;
       end
       return;
   end
   
   % Create LTISource and check compatibility
   nsys = numel(sysList);
   src = repmat(handle(NaN),nsys,1);
   style = cell(nsys,1);
   for ct=1:length(sysList)
      sysInfo = sysList(ct);
      src(ct) = resppack.ltisource(sysInfo.System, 'Name', sysInfo.Name);
      checkCompatibility(src(ct),'sigma');
      style{ct} = sysInfo.Style;
   end

   h = ltiplot(ax,'sigma',[],[],OptionsObject,cstprefs.tbxprefs);
   
catch ME
   throw(ME)
end


% Set global frequency focus for user-defined range/vector (specifies preferred limits)
if isnumeric(w)
   w = unique(w); % G154921
end
h.setFreqFocus(w,['rad/' TimeUnits]);  % w is in rad/TimeUnit

% Store user specified frequency
if isempty(h.UserSpecifiedFrequency)
    h.UserSpecifiedFrequency = w;
end

% Configure responses
NameForms = {'inv(%s)' , '1+%s' , '1+inv(%s)'};
for ct=1:length(src)
   % Link each response to system source
   r = h.addresponse(src(ct));
   DefinedCharacteristics = src(ct).getCharacteristics('sigma');
   r.setCharacteristics(DefinedCharacteristics);
   % Handle special types
   if type>0
      % Doctor response name
      r.Name = sprintf(NameForms{type},r.Name);
      src(ct).Name = r.Name;  % used by data tips
   end
   r.DataFcn =  {'sigma' src(ct) r w type};
   % Styles and preferences
   initsysresp(r,'sigma',h.Options,style{ct})
end


% Draw now
if strcmp(h.AxesGrid.NextPlot,'replace')
   h.Visible = 'on';  % new plot created with Visible='off'
else
   draw(h)  % hold mode
end

% Right-click menus
ltiplotmenu(h,'sigma');

if nargout
   h0 = h;
end


